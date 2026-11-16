"""Directional observed connector costs; committed only after robot recovery."""
import math
import numpy as np


class RouteExperience:
    def __init__(self,capacity=8192):
        self.capacity=capacity
        self.edges={}
        self.pending={}
        self.last=None

    @staticmethod
    def cell(point):
        return tuple(map(int,np.floor(np.asarray(point)/[.5,.5,.25])))

    def observe(self,sample):
        if not sample.position_valid:
            self.last=None
            return
        current=(float(sample.stamp_s),np.asarray(sample.position,dtype=float)-[0.,0.,.44],float(sample.dose_usv))
        if self.last is not None:
            stamp,point,dose=self.last
            dt=current[0]-stamp
            distance=float(np.linalg.norm(current[1]-point))
            start,end=self.cell(point),self.cell(current[1])
            delta=current[2]-dose
            if (0.<dt<=1.5 and .04<=distance<=.35 and 0.<=delta and start!=end
                    and max(abs(a-b) for a,b in zip(start,end))<=1):
                key=(start,end)
                if key in self.pending or len(self.pending)<self.capacity:
                    value=self.pending.setdefault(key,[0,0.,0.,0.,0.])
                    seconds=dt/distance
                    exposure=delta/distance
                    value[0]+=1
                    value[1]+=seconds
                    value[2]+=seconds**2
                    value[3]+=exposure
                    value[4]+=exposure**2
        self.last=current

    def commit(self):
        for key,value in self.pending.items():
            if key in self.edges or len(self.edges)<self.capacity:
                old=self.edges.setdefault(key,[0,0.,0.,0.,0.])
                for i,v in enumerate(value):
                    old[i]+=v
        self.pending={}
        self.last=None

    def bounds(self,start,end,distance):
        entry=self.edges.get((self.cell(start),self.cell(end)))
        if entry is None or entry[0]<2:
            return 0.,0.
        n,seconds,seconds2,dose,dose2=entry
        def upper(total,squared):
            mean=total/n
            variance=max(0.,(squared-total*total/n)/(n-1))
            return (mean+2.*math.sqrt(variance))*distance
        return upper(seconds,seconds2),upper(dose,dose2)

    def export(self):
        return [dict(start=list(a),end=list(b),statistics=value) for (a,b),value in sorted(self.edges.items())]

    def load(self,rows):
        self.edges={(tuple(r['start']),tuple(r['end'])):list(r['statistics']) for r in rows[:self.capacity]}
        self.pending={}
        self.last=None

    def metrics(self):
        return dict(directed_edges=len(self.edges),observations=sum(v[0] for v in self.edges.values()),
            pending_edges=len(self.pending),scope='Recovered robot detector windows; empirical upper cost only, never a dose discount')
