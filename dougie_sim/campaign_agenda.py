"""Persistent 3-D exploration objectives, based solely on observed support."""
import math
from collections import Counter


class CampaignAgenda:
    def __init__(self):
        self.attempts = {}
        self.connectors = Counter()
        self.current_region = None
        self.active = None

    @staticmethod
    def region(point):
        return (math.floor(point[0]/3.),math.floor(point[1]/3.),math.floor((point[2]+.1)/.4))

    def observe_position(self,position):
        region=self.region(position)
        if self.current_region is not None and region!=self.current_region:
            self.connectors[(self.current_region,region)]+=1
        self.current_region=region

    def begin(self,point,support_count):
        region=self.region(point)
        if self.active and self.active['region']==region:
            return
        self.finish(support_count)
        self.active=dict(region=region,support_count=support_count)

    def finish(self,support_count):
        if self.active is None:
            return
        region=self.active['region']
        item=self.attempts.setdefault(region,dict(attempts=0,productive=0,new_support=0))
        gain=max(0,int(support_count)-self.active['support_count'])
        item['attempts']+=1
        item['productive']+=int(gain>=3)
        item['new_support']+=gain
        self.active=None

    def choose(self,ranked,nodes,durations,doses,returns):
        """Rank only pre-admitted goals; no predicted dose or reserve is changed."""
        groups={}
        for score,key,gain in ranked:
            groups.setdefault(self.region(nodes[key]),[]).append((score,key,gain))
        def priority(region):
            choices=groups[region]
            best=max(choices)
            history=self.attempts.get(region,{})
            failures=max(0,history.get('attempts',0)-history.get('productive',0))
            # A region with several distinct nearby frontiers amortizes access.
            spread=min(1.5,1.+.1*max(0,len(choices)-1))
            travel=min(durations[k] for _,k,_ in choices)
            exposure=min(doses[k]+returns[k] for _,k,_ in choices)
            return best[0]*spread/(1.+.35*failures+.001*travel+exposure),region
        region=max(groups,key=priority)
        return max(groups[region])

    def screening_candidates(self,keys,nodes,length,gain,already_checked,limit=8):
        """A bounded shortlist spanning observed XYZ regions and useful views."""
        remaining=[k for k in keys if k not in already_checked]
        near=sorted(remaining,key=lambda k:(length[k],k))
        useful=sorted(remaining,key=lambda k:(-gain[k]/(1.+length[k]),k))
        selected=list(dict.fromkeys(near[:2]+useful[:2]))
        regions={self.region(nodes[k]) for k in selected}
        for key in useful:
            region=self.region(nodes[key])
            if region not in regions:
                selected.append(key)
                regions.add(region)
                if len(selected)>=limit:
                    break
        for key in near:
            if len(selected)>=limit:
                break
            if key not in selected:
                selected.append(key)
        return selected[:limit]

    @staticmethod
    def can_wait_for_screening(sample_upper,return_reserve,rate_upper,budget,
                               stamp,return_seconds,wait_allowance,deadline,wait_s=6.):
        """Extra simulated waiting is allowed only inside the existing reserve."""
        return (sample_upper+return_reserve+rate_upper*wait_s/3600.<budget
            and stamp+wait_s+return_seconds+wait_allowance+15.<deadline)

    def select_tour(self,ranked,terrain,rates,first_time,first_dose,home_dose,
                    remaining,margin,wait_s,gain_cells,excluded=(),return_times=None):
        """Bounded two-goal lookahead over certified first goals and graph edges.

        The second goal is advisory only. Replanning and full admission occur
        again before it can be dispatched. No floor truth is used.
        """
        from .route_forecast import forecast_tree
        top=sorted(ranked,reverse=True)[:6]
        best=None
        for _,first,first_gain in top:
            if first_gain<=0 or first in excluded:
                continue
            _,_,_,parent=terrain.routes(first,rates,excluded=excluded)
            times,doses,_=forecast_tree(terrain,parent,rates)
            for _,second,second_gain in top:
                if first==second or second not in parent or second_gain<=0 or math.dist(terrain.nodes[first],terrain.nodes[second])<.6:
                    continue
                if self.region(terrain.nodes[first])!=self.region(terrain.nodes[second]):
                    continue
                first_cells=gain_cells(first)
                second_cells=gain_cells(second)
                additional=len(second_cells-first_cells)
                if not additional:
                    continue
                dose=first_dose[first]+doses[second]+home_dose[second]+margin
                dose+=(rates[first]+rates[second])*wait_s/3600.
                if dose>remaining:
                    continue
                duration=first_time[first]+times[second]+2*wait_s+(return_times[second] if return_times is not None else 0.)
                score=(first_gain+additional)/(.1+duration/60.+2*dose/.1)
                candidate=(score,first,second,dose,duration,additional)
                if best is None or candidate[0]>best[0]:
                    best=candidate
        return best

    def export(self):
        return dict(attempts=[dict(region=list(k),**v) for k,v in sorted(self.attempts.items())],
            connectors=[dict(start=list(a),end=list(b),count=n) for (a,b),n in sorted(self.connectors.items())])

    def load(self,data):
        self.attempts={tuple(v['region']):{k:int(v[k]) for k in ('attempts','productive','new_support')} for v in data.get('attempts',[])}
        self.connectors=Counter({(tuple(v['start']),tuple(v['end'])):int(v['count']) for v in data.get('connectors',[])})
        self.current_region=self.active=None

    def metrics(self):
        return dict(regions=len(self.attempts),attempts=sum(v['attempts'] for v in self.attempts.values()),
            productive_attempts=sum(v['productive'] for v in self.attempts.values()),
            connector_transitions=sum(self.connectors.values()),active_region=list(self.active['region']) if self.active else None,
            scope='Observed 3-D regions; heuristic ranking only, not a safety or completion criterion')
