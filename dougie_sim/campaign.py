"""Radiation lifetime and retained, independently evolving blind/aware campaigns."""
from copy import deepcopy
import json
import math
from pathlib import Path


class RadiationLifetime:
    def __init__(self,limit):
        if isinstance(limit,bool) or not isinstance(limit,(int,float)) or not math.isfinite(limit) or limit<=0:
            raise ValueError('robot lifetime dose must be positive and finite')
        self.limit,self.dose,self.death=limit,0.,None

    def update(self,dose,stamp,position):
        if not math.isfinite(dose) or dose+1e-12<self.dose:
            raise ValueError('a robot dose cannot be reset in place')
        self.dose=dose
        if self.death is None and dose>=self.limit:
            self.death=dict(time_s=float(stamp),position=list(map(float,position)))
        return self.death is not None

    def payload(self,robot_id=1):
        return dict(robot_id=robot_id,limit_usv=self.limit,absorbed_usv=self.dose,
            remaining_usv=max(0.,self.limit-self.dose),remaining_fraction=max(0.,1.-self.dose/self.limit),
            alive=self.death is None,death=self.death,
            interpretation='Indicative simulation lifetime budget; not a measured Go2 electronics tolerance')


class RadiationCampaign:
    def __init__(self,limit=2.,sorties=6,reserve_fraction=.8,identifier=1):
        RadiationLifetime(limit)
        if type(sorties) is not int or not 1<=sorties<=100 or not isinstance(reserve_fraction,(int,float)) or not math.isfinite(reserve_fraction) or not 0<reserve_fraction<1:
            raise ValueError('invalid campaign sortie count or survival fraction')
        self.limit,self.max_sorties,self.reserve_fraction,self.id=limit,sorties,reserve_fraction,identifier
        self.status,self.reason='active',''
        self.history=[]
        self.protocol=None
        self.modes={m:dict(wrecks=[],previous_directory=None,total_dose_usv=0.,total_time_s=0.,total_distance_m=0.,
            robots_lost=0,robots_recovered=0,sorties=0,explored_percent=0.,last_outcome='not deployed',gp_updates=0,mapped_support_nodes=0) for m in ('blind','aware')}

    def prepare(self,output,protocol=None):
        from .wrecks import origin_blocked
        if self.status!='active':
            raise ValueError('campaign has ended; start a new campaign')
        if self.protocol is not None and protocol is not None and self.protocol!=protocol:
            self.status,self.reason='halted','runtime or mission configuration changed; start a new campaign'
            raise ValueError(self.reason)
        self.protocol=protocol
        for mode,state in self.modes.items():
            if origin_blocked(state['wrecks']):
                self.status,self.reason='halted',f'{mode} origin clearance blocked by a disabled robot'
                raise ValueError(self.reason)
        inputs={}
        for mode,state in self.modes.items():
            inputs[mode]=dict(campaign_id=self.id,sortie=len(self.history)+1,robot_id=state['sorties']+1,
                lifetime_dose_usv=self.limit,reserve_fraction=self.reserve_fraction,origin=[0.,0.,0.],
                previous_directory=state['previous_directory'],wrecks=state['wrecks'],
                prior_dose_usv=state['total_dose_usv'],prior_time_s=state['total_time_s'],prior_distance_m=state['total_distance_m'])
            path=Path(output)/f'{mode}_campaign_input.json'
            path.write_text(json.dumps(inputs[mode],allow_nan=False)+'\n')
        return {m:Path(output)/f'{m}_campaign_input.json' for m in inputs}

    def finish(self,record,output,summaries):
        if any(r['run_id']==record['id'] for r in self.history):
            return
        results={}
        for mode,state in self.modes.items():
            summary=summaries[mode]
            state['sorties']+=1
            for source,target in (('true_dose_usv','total_dose_usv'),('distance_3d_m','total_distance_m')):
                value=summary.get(source,0.)
                if isinstance(value,(int,float)) and math.isfinite(value) and value>=0:
                    state[target]+=value
            elapsed=summary.get('simulation',{}).get('simulation_time_s',0.)
            if isinstance(elapsed,(int,float)) and math.isfinite(elapsed) and elapsed>=0:
                state['total_time_s']+=elapsed
            state['explored_percent']=summary.get('comparison_survey',{}).get('explored_percent',state['explored_percent'])
            state['gp_updates']=summary.get('gp',{}).get('updates',0)
            state['mapped_support_nodes']=summary.get('supported_graph_nodes',0)
            state['campaign_agenda']=summary.get('campaign_agenda')
            state['route_experience']=summary.get('route_experience')
            life=summary.get('robot_lifetime') or {}
            recovered=(life.get('alive') is True and summary.get('end_state') in ('returned','finished')
                and not summary.get('error') and summary.get('localization_evaluation',{}).get('true_home_distance_m',math.inf)<=.4)
            if life.get('alive') is False:
                try:
                    wreck=json.loads((Path(output)/mode/'dead_robot.json').read_text())
                    state['wrecks'].append(wreck)
                    state['robots_lost']+=1
                    outcome='destroyed_radiation'
                except (OSError,ValueError):
                    outcome='failure_artifacts_missing'
                    self.status,self.reason='halted',f'{mode} failure geometry is unavailable'
            elif recovered:
                state['robots_recovered']+=1
                outcome='recovered_for_swap'
            else:
                outcome='stranded_or_interrupted'
                self.status,self.reason='halted',f'{mode} robot did not return alive; no automatic in-field dose reset'
            directory=Path(output)/mode
            if not all((directory/name).is_file() for name in ('campaign_memory.npz','gp_posterior.npz','coverage_reference.npz','minimap_final.npz')) or summary.get('error'):
                self.status,self.reason='halted',f'{mode} mission memory or result is incomplete'
            else:
                state['previous_directory']=str(directory.resolve())
            state['last_outcome']=outcome
            shared=summary.get('shared_routes',{})
            state['shared_recovered_routes']=shared.get('recovered_routes',0)+(1 if recovered else 0)
            state['shared_route_edges']=shared.get('usable_graph_edges',0)
            results[mode]=dict(outcome=outcome,recovered=recovered,robot_lifetime=life,
                explored_percent=state['explored_percent'],total_dose_usv=state['total_dose_usv'],wrecks=len(state['wrecks']),
                gp_updates=state['gp_updates'],mapped_support_nodes=state['mapped_support_nodes'],campaign_agenda=state['campaign_agenda'],route_experience=state['route_experience'],
                shared_recovered_routes=state['shared_recovered_routes'],shared_route_edges=state['shared_route_edges'])
        self.history.append(dict(run_id=record['id'],sortie=len(self.history)+1,results=results))
        from .wrecks import origin_blocked
        blocked=[m for m,s in self.modes.items() if origin_blocked(s['wrecks'])]
        if blocked:
            self.status,self.reason='halted',f'{", ".join(blocked)} origin clearance blocked by a wreck'
        elif self.status=='active' and any(summaries[m].get('frontier_complete') and results[m]['recovered'] for m in self.modes):
            self.status,self.reason='complete','survey exhausted and robot recovered; paired campaign concluded'
        elif self.status=='active' and len(self.history)>=self.max_sorties:
            self.status,self.reason='sortie_limit','configured mission-package sortie count reached'
        record['campaign_result']=results

    def save(self):
        return deepcopy(dict(schema_version=1,id=self.id,limit=self.limit,max_sorties=self.max_sorties,
            reserve_fraction=self.reserve_fraction,status=self.status,reason=self.reason,history=self.history,modes=self.modes,protocol=self.protocol))

    @classmethod
    def load(cls,data):
        if data.get('schema_version')!=1:
            raise ValueError('unsupported campaign state')
        value=cls(data['limit'],data['max_sorties'],data['reserve_fraction'],data['id'])
        for name in ('status','reason','history','modes'):
            setattr(value,name,deepcopy(data[name]))
        value.protocol=data.get('protocol')
        return value

    def payload(self):
        return dict(enabled=True,id=self.id,limit_usv=self.limit,max_sorties=self.max_sorties,
            planning_budget_usv=self.limit*self.reserve_fraction,sorties_finished=len(self.history),
            status=self.status,reason=self.reason,history=self.history,
            modes={m:{k:v for k,v in state.items() if k not in ('wrecks','previous_directory')}|dict(wrecks=len(state['wrecks']))
                for m,state in self.modes.items()},
            scope='Independent policy campaigns: GP/observed maps persist; replacement only after recovery or loss. Wrecks remain physical obstacles. Fixed reference denominator; actual active-sortie exposure.')
