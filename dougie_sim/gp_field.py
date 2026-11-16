"""Bounded, continuous display sampling of onboard GP predictions only."""
import numpy as np
from scipy.spatial import cKDTree


def observed_field_queries(terrain, detector_positions=(), capacity=8192, spacing=.25):
    """Dense, stable coverage of observed supports; no evaluator layout queries."""
    if capacity < 1 or not np.isfinite(spacing) or spacing <= 0:
        raise ValueError('invalid GP field query budget or spacing')
    priority = np.unique(np.asarray(detector_positions,dtype=float).reshape(-1,3),axis=0)
    priority = priority[np.isfinite(priority).all(axis=1)][:capacity]
    if hasattr(terrain,'support_returns'):
        support = np.asarray(list(terrain.support_returns.values()),dtype=float).reshape(-1,3)
    elif getattr(terrain,'surfaces',None):
        support = np.array([[(k[0]+.5)*terrain.resolution,(k[1]+.5)*terrain.resolution,z]
                            for k,z in terrain.surfaces.items()])
    else:
        support = np.asarray(list(terrain.nodes.values()),dtype=float).reshape(-1,3)
    support = support[np.isfinite(support).all(axis=1)]
    remaining = capacity-len(priority)
    if not len(support) or remaining <= 0:
        return priority
    size = np.array([spacing,spacing,.1])
    cells = np.floor(support/size+1e-8).astype(np.int64)
    distance = np.linalg.norm(support-(cells+.5)*size,axis=1)
    order = np.lexsort((distance,cells[:,2],cells[:,1],cells[:,0]))
    ordered = cells[order]
    first = np.r_[True,np.any(np.diff(ordered,axis=0),axis=1)]
    # Retain an actual return in each cell, not an invented floor/mesh height.
    points = support[order[first]]+[0.,0.,.44]
    if len(points) > remaining:
        points = points[np.linspace(0,len(points)-1,remaining).astype(int)]
    return np.unique(np.concatenate((priority,points)),axis=0)


def interpolate_field(query, xyz, mean, std, upper, radius=(.35,.35,.18)):
    """Compact C2 Shepard weights; uncertainty is a mixture, not precision fusion.

    This interpolates exported display estimates, not new detector observations.
    Neighbours enter at zero weight. Uncovered cells remain NaN, never zero.
    """
    query = np.asarray(query,dtype=float).reshape(-1,3)
    xyz = np.asarray(xyz,dtype=float).reshape(-1,3)
    mean,std,upper = (np.asarray(a,dtype=float).reshape(-1) for a in (mean,std,upper))
    scale = np.asarray(radius,dtype=float)
    if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0):
        raise ValueError('invalid interpolation radius')
    if not (len(xyz)==len(mean)==len(std)==len(upper)):
        raise ValueError('inconsistent exported GP field arrays')
    valid = np.isfinite(xyz).all(axis=1)&np.isfinite(mean)&np.isfinite(std)&np.isfinite(upper)&(mean>=0)&(std>=0)&(upper>=mean)
    xyz,mean,std,upper = xyz[valid],mean[valid],std[valid],upper[valid]
    output = np.full((len(query),3),np.nan)
    nearest,counts = np.full(len(query),np.inf),np.zeros(len(query),dtype=int)
    if not len(xyz):
        return output[:,0],output[:,1],output[:,2],nearest,counts
    tree = cKDTree(xyz/scale)
    nearest = tree.query(query/scale)[0]
    neighbours = tree.query_ball_point(query/scale,1.)
    for i,indices in enumerate(neighbours):
        if not indices:
            continue
        indices = np.asarray(indices)
        distance = np.linalg.norm((xyz[indices]-query[i])/scale,axis=1)
        weight = np.maximum(0.,1.-distance)**4*(1.+4.*distance)/np.maximum(distance**2,1e-24)
        total = weight.sum()
        if total <= 0:
            continue
        weight /= total
        latent = np.log(np.maximum(mean[indices],1e-30))
        centre = weight@latent
        output[i] = (weight@mean[indices],
            np.sqrt(weight@(std[indices]**2+(latent-centre)**2)),weight@upper[indices])
        counts[i] = int((weight>0).sum())
    return output[:,0],output[:,1],output[:,2],nearest,counts
