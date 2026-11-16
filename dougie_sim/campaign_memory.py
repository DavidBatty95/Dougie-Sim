"""Retain onboard map/field knowledge, never a dead robot's dose allowance."""
from pathlib import Path
import json

import numpy as np
from scipy.spatial import cKDTree


def save_memory(explorer,path,recovered_route=None):
    terrain=explorer.terrain
    support=(list(terrain.support_returns.values()) if hasattr(terrain,'support_returns') else
        [[(k[0]+.5)*terrain.resolution,(k[1]+.5)*terrain.resolution,z] for k,z in terrain.surfaces.items()])
    routes=list(terrain.shared_routes)
    if recovered_route is not None:
        route=np.asarray(recovered_route,dtype=float).reshape(-1,3)
        if len(route)>1 and np.isfinite(route).all():
            routes.append(route)
    values=dict(schema_version=2,supports=np.asarray(support).reshape(-1,3),
        hits=np.asarray(list(terrain.hits.values())).reshape(-1,3),
        free_voxels=np.asarray(list(terrain.free_voxels)).reshape(-1,3),
        visited=np.asarray([terrain.nodes[k] for k in explorer.visited if k in terrain.nodes]).reshape(-1,3),
        exposure_cells=np.asarray(list(explorer.utility.exposure),dtype=np.int64).reshape(-1,3),
        exposure_seconds=np.asarray(list(explorer.utility.exposure.values())),
        route_offsets=np.r_[0,np.cumsum([len(r) for r in routes],dtype=np.int64)],
        route_points=np.concatenate(routes) if routes else np.empty((0,3)))
    if hasattr(terrain,'occupancy'):
        values.update(occupancy_keys=np.asarray(list(terrain.occupancy.evidence),dtype=np.int64).reshape(-1,3),
            occupancy_values=np.asarray(list(terrain.occupancy.evidence.values()),dtype=np.int8),
            coarse_keys=np.asarray(list(terrain.occupancy.coarse_free),dtype=np.int64).reshape(-1,3),
            coarse_values=np.asarray(list(terrain.occupancy.coarse_free.values()),dtype=np.int8))
    agenda=getattr(explorer,'campaign_agenda',None)
    if agenda is not None:
        agenda.finish(len(getattr(explorer,'observed_support_keys',())))
        values['campaign_agenda_json']=np.asarray(json.dumps(agenda.export()))
    views=getattr(explorer,'frontier_views',None)
    if views is not None:
        values['frontier_views_json']=np.asarray(json.dumps(views.export()))
    experience=getattr(terrain,'route_experience',None)
    if experience is not None:
        if recovered_route is not None:
            experience.commit()
        values['route_experience_json']=np.asarray(json.dumps(experience.export()))
    np.savez_compressed(path,**values)


def restore_memory(explorer,directory,origin):
    directory=Path(directory)
    with np.load(directory/'campaign_memory.npz',allow_pickle=False) as data:
        if int(data['schema_version']) not in (1,2):
            raise ValueError('unsupported mission memory')
        explorer.terrain.update(data['hits'],data['supports'])
        explorer.terrain.free_voxels.update(map(tuple,data['free_voxels']))
        if hasattr(explorer.terrain,'occupancy') and 'occupancy_keys' in data:
            explorer.terrain.occupancy.evidence=dict(zip(map(tuple,data['occupancy_keys']),map(int,data['occupancy_values'])))
            explorer.terrain.occupancy.coarse_free=dict(zip(map(tuple,data['coarse_keys']),map(int,data['coarse_values'])))
        explorer.terrain.build_graph()
        keys=list(explorer.terrain.nodes)
        if keys and len(data['visited']):
            points=np.asarray([explorer.terrain.nodes[k] for k in keys])
            distance,index=cKDTree(points).query(data['visited'])
            explorer.visited={keys[i] for d,i,p in zip(distance,index,data['visited'])
                if d<=.4 and abs(points[i,2]-p[2])<=.18}
        explorer.utility.exposure={tuple(map(int,k)):float(v) for k,v in zip(data['exposure_cells'],data['exposure_seconds'])}
        if 'route_offsets' in data:
            offsets=data['route_offsets']
            explorer.terrain.shared_routes=[data['route_points'][a:b].copy() for a,b in zip(offsets[:-1],offsets[1:])]
            explorer.terrain.refresh_shared_route_edges()
        if 'campaign_agenda_json' in data and getattr(explorer,'campaign_agenda',None) is not None:
            explorer.campaign_agenda.load(json.loads(str(data['campaign_agenda_json'].item())))
        if 'frontier_views_json' in data and getattr(explorer,'frontier_views',None) is not None:
            explorer.frontier_views.load(json.loads(str(data['frontier_views_json'].item())))
        if 'route_experience_json' in data and getattr(explorer.terrain,'route_experience',None) is not None:
            explorer.terrain.route_experience.load(json.loads(str(data['route_experience_json'].item())))
        if hasattr(explorer,'sampled_height_bands'):
            explorer.sampled_height_bands={round(float(p[2])/.2) for p in data['visited']}
        if hasattr(explorer,'observed_support_keys'):
            explorer.observed_support_keys={(k[0],k[1],round(float(p[2])/.2)) for k,p in explorer.terrain.nodes.items()}
    explorer.home=np.asarray(origin,dtype=float)
    if hasattr(explorer,'last_support_progress_s'):
        explorer.last_support_progress_s=0.
