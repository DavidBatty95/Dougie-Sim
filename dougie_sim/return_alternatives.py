"""Compare observed return paths under the same upper-exposure model."""
import math
import numpy as np

from .route_forecast import forecast_polyline,forecast_tree


def best_return_route(terrain,root,home,position,base_parent,rates,gp,smoother,
                      *,excluded=(),start_yaw=None,local_upper=0.):
    """Return the lowest certified upper-dose option; preserve physical admission.

    Distinct time/mean-cost, geometry and upper-dose graph parents are only
    candidates. Every alternative is smoothed with the ordinary return contract
    and evaluated with the same GP upper field, turning, and current measurement.
    Experienced graph costs may only increase this forecast. No truth is used.
    """
    candidates=[('current',base_parent)]
    for name,values,scale in (('geometric',None,50.),('upper_dose',rates,1e-6)):
        parent=terrain.routes(root,values,rate_scale=scale,excluded=excluded)[3]
        if home in parent:
            candidates.append((name,parent))
    seen=set()
    variants=[]
    for name,parent in candidates:
        if home not in parent:
            continue
        keys=terrain.path(parent,home)
        identity=tuple(keys)
        if identity in seen:
            continue
        seen.add(identity)
        raw=[terrain.nodes[k].copy() for k in keys]
        checked=smoother([position,*raw],returning=True)
        if not checked.points:
            continue
        query=np.asarray(checked.points)+[0.,0.,.44]
        mean,_,upper=gp.predict(query)
        policy=np.maximum(mean,upper)
        policy[0]=max(float(policy[0]),local_upper)
        duration,dose=forecast_polyline(checked.points,policy,start_yaw=start_yaw)
        if getattr(terrain,'route_experience',None) is not None and terrain.route_experience.edges:
            grid_time,grid_dose,_=forecast_tree(terrain,parent,rates)
            duration=max(duration,grid_time[home])
            dose=max(dose,grid_dose[home])
        if not math.isfinite(dose) or not math.isfinite(duration):
            continue
        variants.append(dict(family=name,raw=raw,motion=checked,duration_s=duration,upper_dose_usv=dose,
            peak_rate_usv_h=float(np.max(policy))))
    if not variants or variants[0]['family']!='current':
        return None
    baseline=variants[0]
    best=min(variants,key=lambda item:(item['upper_dose_usv'],item['duration_s']))
    return dict(best=best,baseline=baseline,variants=variants)
