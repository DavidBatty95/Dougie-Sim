"""Small rotation helpers shared by sensor publication and measured extrinsics."""
import math


def matrix_to_quat_xyzw(rotation):
    m00,m01,m02 = rotation[0]
    m10,m11,m12 = rotation[1]
    m20,m21,m22 = rotation[2]
    trace = m00+m11+m22
    if trace>0.:
        s = math.sqrt(trace+1.)*2.
        q = ((m21-m12)/s,(m02-m20)/s,(m10-m01)/s,.25*s)
    elif m00>m11 and m00>m22:
        s = math.sqrt(1.+m00-m11-m22)*2.
        q = (.25*s,(m01+m10)/s,(m02+m20)/s,(m21-m12)/s)
    elif m11>m22:
        s = math.sqrt(1.+m11-m00-m22)*2.
        q = ((m01+m10)/s,.25*s,(m12+m21)/s,(m02-m20)/s)
    else:
        s = math.sqrt(1.+m22-m00-m11)*2.
        q = ((m02+m20)/s,(m12+m21)/s,.25*s,(m10-m01)/s)
    return tuple(map(float,q))
