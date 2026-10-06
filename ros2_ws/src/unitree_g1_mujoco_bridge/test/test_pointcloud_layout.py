import struct
from types import SimpleNamespace as S

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.pointcloud_layout import xyz_intensity


@pytest.mark.parametrize('endian',['<','>'])
def test_hardware_padded_organized_cloud_with_extra_fields(endian):
    data = bytearray(160)
    for row in range(2):
        for col in range(2):
            start = row*80+col*32
            struct.pack_into(endian+'fff',data,start,float(row),float(col),1.)
            data[start+16] = 42
    msg = S(width=2,height=2,point_step=32,row_step=80,is_bigendian=endian=='>',data=data,
            fields=[S(name=name,offset=offset,datatype=kind,count=1) for name,offset,kind in
                    [('intensity',16,2),('z',8,7),('x',0,7),('y',4,7),('tag',17,2)]])
    np.testing.assert_allclose(xyz_intensity(msg),[[0,0,1,42],[0,1,1,42],[1,0,1,42],[1,1,1,42]])
    msg.data = data[:-1]
    with pytest.raises(ValueError,match='stride'):
        xyz_intensity(msg)


def test_missing_coordinates_rejected_instead_of_reinterpreting_bytes():
    msg = S(width=1,height=1,point_step=16,row_step=16,is_bigendian=False,data=bytes(16),fields=[])
    with pytest.raises(ValueError,match='field'):
        xyz_intensity(msg)
