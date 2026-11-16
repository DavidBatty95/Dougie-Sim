"""Bounded observed-support measurement probes for uncertain blocked frontiers."""
import numpy as np
from scipy.spatial import cKDTree


def target_still_uncertain(point,nodes,frontiers,status,uncertainty,blocked):
    """Revalidate the purpose of an in-flight probe without resetting its goal."""
    return any(status.get(k) in blocked and uncertainty.get(k,0.)>1.
        and .35<=np.linalg.norm(np.asarray(point)-nodes[k])<=1.25
        and abs(point[2]-nodes[k][2])<=.35 for k in frontiers)


def probe_candidates(nodes,frontiers,status,uncertainty,length,utility,available,blocked,*,limit=8):
    """Choose nearby, unmeasured stand-offs; caller must apply normal admission.

    Target selection uses planner GP/support only; no evaluator field or coverage.
    A probe is never itself treated as resolving a geometric frontier.
    """
    targets=[k for k in frontiers if status.get(k) in blocked and uncertainty.get(k,0.)>1.]
    if not targets:
        return []
    tree=cKDTree(np.asarray([nodes[k] for k in targets]))
    eligible=[k for k in nodes if k not in frontiers and status.get(k)==available and .25<=length.get(k,float('inf'))<=6.]
    if not eligible:
        return []
    neighbourhoods=tree.query_ball_point([nodes[k] for k in eligible],1.25)
    ranked=[]
    for key,indices in zip(eligible,neighbourhoods):
        if utility.neighbourhood_seconds(nodes[key]+[0.,0.,.44])>=.25:
            continue
        values=[]
        for i in indices:
            target=targets[i]
            d=float(np.linalg.norm(nodes[key]-nodes[target]))
            if .35<=d<=1.25 and abs(nodes[key][2]-nodes[target][2])<=.35:
                values.append(float(uncertainty[target])/(1.+length[key]+d))
        if values:
            ranked.append((max(values),key))
    ranked.sort(reverse=True)
    chosen=[]
    for _,key in ranked:
        if all(np.linalg.norm(nodes[key]-nodes[other])>=.5 for other in chosen):
            chosen.append(key)
            if len(chosen)>=limit:
                break
    return chosen
