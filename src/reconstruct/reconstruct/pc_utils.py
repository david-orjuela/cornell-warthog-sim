#!/usr/bin/env python3
"""Point-cloud conversion helpers for ROS 2.

This module deliberately avoids ros_numpy, which is primarily a ROS 1 dependency.
"""

from __future__ import annotations

import copy
from typing import Optional

import numpy as np
import open3d as o3d
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def o3dpc_to_pointcloud2(
    cloud: o3d.geometry.PointCloud,
    frame_id: str,
    stamp: Optional[Time] = None,
) -> PointCloud2:
    """Convert an Open3D point cloud to a ROS 2 ``sensor_msgs/PointCloud2``.

    RGB is packed into the conventional PCL-compatible float32 ``rgb`` field.
    The function makes no assumptions about the node clock; callers should pass
    the source sensor timestamp whenever possible.
    """
    points = np.asarray(copy.deepcopy(cloud.points), dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        points = np.empty((0, 3), dtype=np.float32)

    has_color = len(cloud.colors) == len(cloud.points) and len(points) > 0
    point_count = int(points.shape[0])

    if has_color:
        dtype = np.dtype(
            [
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("rgb", "<f4"),
            ]
        )
    else:
        dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")])

    packed = np.zeros(point_count, dtype=dtype)
    if point_count:
        packed["x"] = points[:, 0]
        packed["y"] = points[:, 1]
        packed["z"] = points[:, 2]

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    point_step = 12
    if has_color:
        colors = np.asarray(copy.deepcopy(cloud.colors), dtype=np.float64)
        colors = np.clip(np.rint(colors * 255.0), 0, 255).astype(np.uint32)
        rgb_uint32 = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
        packed["rgb"] = rgb_uint32.view(np.float32)
        fields.append(
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1)
        )
        point_step = 16

    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp if stamp is not None else Time()
    msg.height = 1
    msg.width = point_count
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * point_count
    msg.data = packed.tobytes()
    msg.is_dense = bool(point_count == 0 or np.isfinite(points).all())
    return msg
