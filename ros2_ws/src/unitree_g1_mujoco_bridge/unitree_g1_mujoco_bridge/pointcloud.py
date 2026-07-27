import math
import struct
from collections.abc import Iterable

from sensor_msgs.msg import PointCloud2


def iter_xyz(msg: PointCloud2) -> Iterable[tuple[float, float, float]]:
    """Yield finite XYZ float32 points from a PointCloud2 message.

    The simulated Livox and derived obstacle clouds use standard little-endian
    float32 fields. Invalid layouts produce no points instead of failing inside
    a callback, which keeps sensor pipelines alive while logging remains local
    to the consuming node.
    """
    offsets = {field.name: field.offset for field in msg.fields}
    if not {"x", "y", "z"}.issubset(offsets) or msg.point_step <= 0:
        return

    x_offset = offsets["x"]
    y_offset = offsets["y"]
    z_offset = offsets["z"]
    max_offset = max(x_offset, y_offset, z_offset) + 4
    if msg.point_step < max_offset:
        return

    data = msg.data
    for base in range(0, len(data) - max_offset + 1, msg.point_step):
        x = struct.unpack_from("<f", data, base + x_offset)[0]
        y = struct.unpack_from("<f", data, base + y_offset)[0]
        z = struct.unpack_from("<f", data, base + z_offset)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            yield x, y, z
