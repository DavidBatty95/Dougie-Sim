"""Persistent, bounded run-to-run policy tuning from completed simulation trials.

The learner receives evaluator outcomes only after a mission. It never supplies
source positions, true field queries or reference coverage to a running planner.
"""
from copy import deepcopy
import math
from statistics import mean

DEFAULT_WEIGHTS = dict(route_dose_weight=1.,frontier_dose_weight=2.)
BOUNDS = (.25,16.)
DIRECTIONS = (('route_dose_weight',2.),('route_dose_weight',.5),
              ('frontier_dose_weight',2.),('frontier_dose_weight',.5))


def planning_weights(route=1.,frontier=2.):
    values = dict(route_dose_weight=float(route),frontier_dose_weight=float(frontier))
    if not all(math.isfinite(v) and BOUNDS[0]<=v<=BOUNDS[1] for v in values.values()):
        raise ValueError('planning dose weights must be finite and between .25 and 16')
    return values


def evaluate_attempt(record,pair,milestone=75.,progress_tolerance=.5):
    result = dict(eligible=False,reason='',milestone_percent=milestone,
        reference_only=record.get('learning_trial') is None,runs={})
    for mode in ('blind','aware'):
        rows = pair.get(mode,[])
        if not rows:
            result['reason'] = 'missing telemetry'
            return result
        required = ('time_s','true_dose_usv','distance_m','explored_percent')
        if any(not all(isinstance(r.get(k),(int,float)) and math.isfinite(r[k]) and r[k]>=0 for k in required) for r in rows):
            result['reason'] = 'invalid telemetry'
            return result
        if any(b['time_s']<a['time_s'] for a,b in zip(rows,rows[1:])):
            result['reason'] = 'nonmonotonic telemetry clock'
            return result
        crossing = next((r for r in rows if r['explored_percent']>=milestone),None)
        result['runs'][mode] = dict(endpoint={k:rows[-1].get(k) for k in (*required,'frontier_complete','state')},
            crossing={k:crossing.get(k) for k in required} if crossing else None)
    if record['status']!='ended':
        result['reason'] = 'interrupted or nonterminal attempt'
        return result
    if any(record['exit_codes'].get(m)!=0 for m in ('blind','aware')):
        result['reason'] = 'worker failure'
        return result
    for mode in ('blind','aware'):
        outcome = record.get('outcomes',{}).get(mode,{})
        if outcome.get('error') or outcome.get('artifact_error') or outcome.get('clean_return') is False:
            result['reason'] = 'failed or unclean mission'
            return result
        if result['runs'][mode]['endpoint']['state'] not in ('finished','returned'):
            result['reason'] = 'mission did not end normally'
            return result
    blind,aware = (result['runs'][m] for m in ('blind','aware'))
    if not blind['crossing'] or not aware['crossing']:
        result['reason'] = 'fixed exploration milestone not reached by both robots'
        return result
    if abs(blind['crossing']['explored_percent']-aware['crossing']['explored_percent'])>2.:
        result['reason'] = 'milestone crossing overshoots differ by more than 2 percentage points'
        return result
    if aware['endpoint']['explored_percent']+progress_tolerance<blind['endpoint']['explored_percent']:
        result['reason'] = 'aware endpoint explores less than the blind reference'
        return result
    if blind['endpoint'].get('frontier_complete') and not aware['endpoint'].get('frontier_complete'):
        result['reason'] = 'aware mission completion regressed'
        return result
    if min(blind['endpoint']['true_dose_usv'],blind['crossing']['true_dose_usv'])<=1e-12:
        result['reason'] = 'reference dose too small for a stable comparison'
        return result
    result.update(eligible=True,reason='eligible comparable-progress attempt',
        total_dose_ratio=aware['endpoint']['true_dose_usv']/blind['endpoint']['true_dose_usv'],
        milestone_dose_ratio=aware['crossing']['true_dose_usv']/blind['crossing']['true_dose_usv'])
    return result


class PlanningLearner:
    def __init__(self,weights=None,milestone=75.,seeds_per_round=3,base_seed=1,enabled=False):
        if not math.isfinite(milestone) or not 0<milestone<=100 or not 2<=seeds_per_round<=10 or base_seed<0:
            raise ValueError('invalid learning milestone or seed panel')
        weights = weights or DEFAULT_WEIGHTS
        self.best = planning_weights(weights['route_dose_weight'],weights['frontier_dose_weight'])
        self.milestone,self.seeds_per_round,self.base_seed = milestone,seeds_per_round,base_seed
        self.enabled,self.started = enabled,False
        self.generation,self.slot,self.cursor = 0,0,0
        self.candidate = self._candidate()
        self.entries,self.rounds = [],[]

    def _candidate(self):
        for _ in DIRECTIONS:
            name,factor = DIRECTIONS[self.cursor%len(DIRECTIONS)]
            value = self.best[name]*factor
            if BOUNDS[0]<=value<=BOUNDS[1]:
                return dict(self.best,**{name:value})
            self.cursor += 1
        raise RuntimeError('no bounded planning challenger')

    def set_enabled(self,enabled,next_seed):
        if enabled and not self.started:
            self.base_seed = next_seed
        self.enabled = enabled

    def propose(self):
        if not self.enabled:
            return None
        self.started = True
        role = 'incumbent' if self.slot%2==0 else 'challenger'
        return dict(id=f'{self.generation}:{self.slot}',round=self.generation+1,slot=self.slot,
            seed=self.base_seed+self.generation*self.seeds_per_round+self.slot//2,
            role=role,weights=deepcopy(self.best if role=='incumbent' else self.candidate))

    def observe(self,record,result):
        trial = record.get('learning_trial')
        if trial is None or any(e['trial']['id']==trial['id'] for e in self.entries):
            return
        if trial['round']!=self.generation+1 or trial['slot']!=self.slot:
            raise ValueError('out-of-order learning trial')
        result = deepcopy(result)
        if (record.get('seed')!=trial['seed'] or
                record.get('planning_weights',{}).get('aware')!=trial['weights']):
            result.update(eligible=False,reason='recorded seed or planning weights do not match the proposed trial')
        record['learning_result'] = deepcopy(result)
        self.entries.append(dict(run_id=record['id'],trial=deepcopy(trial),result=deepcopy(result),
            protocol=record.get('learning_protocol_sha256')))
        self.slot += 1
        if self.slot<2*self.seeds_per_round:
            return
        panel = self.entries[-2*self.seeds_per_round:]
        same_protocol = bool(panel[0].get('protocol')) and all(e.get('protocol')==panel[0]['protocol'] for e in panel)
        eligible = same_protocol and all(e['result']['eligible'] for e in panel)
        promote = False
        reason = ('runtime or experiment settings changed within the matched-seed panel' if not same_protocol
            else 'one or more interrupted, failed or incomparable attempts')
        statistics = None
        if eligible:
            incumbent,challenger = panel[::2],panel[1::2]
            total_old = [e['result']['runs']['aware']['endpoint']['true_dose_usv'] for e in incumbent]
            total_new = [e['result']['runs']['aware']['endpoint']['true_dose_usv'] for e in challenger]
            milestone_old = [e['result']['runs']['aware']['crossing']['true_dose_usv'] for e in incumbent]
            milestone_new = [e['result']['runs']['aware']['crossing']['true_dose_usv'] for e in challenger]
            progress_ok = all(c['result']['runs']['aware']['endpoint']['explored_percent']+.5>=i['result']['runs']['aware']['endpoint']['explored_percent']
                and (not i['result']['runs']['aware']['endpoint'].get('frontier_complete') or c['result']['runs']['aware']['endpoint'].get('frontier_complete'))
                for i,c in zip(incumbent,challenger))
            baseline_stable = all(
                abs(c['result']['runs']['blind']['endpoint']['true_dose_usv']-i['result']['runs']['blind']['endpoint']['true_dose_usv'])
                    <=.02*i['result']['runs']['blind']['endpoint']['true_dose_usv']
                and abs(c['result']['runs']['blind']['endpoint']['explored_percent']-i['result']['runs']['blind']['endpoint']['explored_percent'])<=.5
                and bool(c['result']['runs']['blind']['endpoint'].get('frontier_complete'))==bool(i['result']['runs']['blind']['endpoint'].get('frontier_complete'))
                for i,c in zip(incumbent,challenger))
            promote = (baseline_stable and progress_ok and all(c<i for i,c in zip(total_old,total_new))
                and mean(total_new)<=.98*mean(total_old) and mean(milestone_new)<=1.02*mean(milestone_old))
            reason = ('challenger passed the matched-seed acceptance checks' if promote else
                'fixed blind reference changed between matched-seed trials' if not baseline_stable else
                'challenger did not improve total dose consistently without progress/milestone regression')
            statistics = dict(incumbent_total_aware_dose_usv=mean(total_old),challenger_total_aware_dose_usv=mean(total_new),
                incumbent_milestone_aware_dose_usv=mean(milestone_old),challenger_milestone_aware_dose_usv=mean(milestone_new),
                paired_seed_dose_savings_usv=[i-c for i,c in zip(total_old,total_new)])
        self.rounds.append(dict(round=self.generation+1,incumbent=deepcopy(self.best),challenger=deepcopy(self.candidate),
            seeds=list(range(self.base_seed+self.generation*self.seeds_per_round,self.base_seed+(self.generation+1)*self.seeds_per_round)),
            promoted=bool(promote),reason=reason,statistics=statistics))
        if promote:
            self.best = deepcopy(self.candidate)
            self.cursor = 0
        else:
            self.cursor += 1
        self.generation += 1
        self.slot = 0
        self.candidate = self._candidate()

    def save(self):
        return deepcopy(dict(schema_version=1,enabled=self.enabled,started=self.started,best=self.best,
            milestone=self.milestone,seeds_per_round=self.seeds_per_round,base_seed=self.base_seed,
            generation=self.generation,slot=self.slot,cursor=self.cursor,candidate=self.candidate,
            entries=self.entries,rounds=self.rounds))

    @classmethod
    def load(cls,data,enabled=False):
        if data.get('schema_version')!=1:
            raise ValueError('unsupported learning state')
        learner = cls(data['best'],data['milestone'],data['seeds_per_round'],data['base_seed'],enabled)
        for name in ('started','generation','slot','cursor','candidate','entries','rounds'):
            setattr(learner,name,deepcopy(data[name]))
        planning_weights(data['candidate']['route_dose_weight'],data['candidate']['frontier_dose_weight'])
        return learner

    def payload(self):
        return dict(self.save(),bounds=list(BOUNDS),minimum_mean_improvement_percent=2.,endpoint_progress_tolerance_percent=.5,
            objective='Actual total aware dose including return, at comparable final progress; fixed-milestone dose is a non-regression check.',
            scope='Sequential simulation policy tuning on matched detector-seed panels. Retained settings are provisional development results, not independent-layout or hardware validation.')
