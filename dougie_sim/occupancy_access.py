"""Exact lookup of compact 3-D occupancy evidence without expanding Python sets."""
import numpy as np


def dense_confirmed_free(evidence,coarse_free,origin,shape):
    """Materialize only the bounded query grid, preserving fine/coarse union.

    Coarse keys represent complete 2x2x2 free blocks; negative keys use floor
    division just like VoxelEvidence. Unknown and weak evidence remain false.
    """
    origin=np.asarray(origin,dtype=int)
    shape=np.asarray(shape,dtype=int)
    grid=np.zeros(tuple(shape),dtype=bool)
    fine=np.asarray([key for key,value in evidence.items() if value<=-3],dtype=int).reshape(-1,3)-origin
    fine=fine[np.all((fine>=0)&(fine<shape),axis=1)]
    grid[tuple(fine.T)]=True
    blocks=np.asarray(list(coarse_free),dtype=int).reshape(-1,3)*2-origin
    for dx in (0,1):
        for dy in (0,1):
            for dz in (0,1):
                indices=blocks+[dx,dy,dz]
                indices=indices[np.all((indices>=0)&(indices<shape),axis=1)]
                grid[tuple(indices.T)]=True
    return grid


class SparseConfirmedFree:
    """Immutable sparse snapshot; retain coarse blocks rather than 8 children."""
    def __init__(self,evidence,coarse_free):
        self.fine=frozenset(key for key,value in evidence.items() if value<=-3)
        self.coarse=frozenset(coarse_free)

    def contains(self,voxels):
        voxels=np.asarray(voxels,dtype=int)
        return np.fromiter(((int(x),int(y),int(z)) in self.fine or
            (int(x)//2,int(y)//2,int(z)//2) in self.coarse for x,y,z in voxels.reshape(-1,3)),
            dtype=bool,count=voxels.size//3).reshape(voxels.shape[:-1])
