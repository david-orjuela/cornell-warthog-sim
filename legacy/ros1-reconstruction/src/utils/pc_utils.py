import copy
import open3d as o3d
import numpy as np
import ros_numpy
import rospy
from sensor_msgs.msg import PointField, PointCloud2


def visualize_point_clouds_with_colors(trunk_component_list):
    pcd_list = []
    colors = np.random.rand(len(trunk_component_list), 3)
    for i, trunk in enumerate(trunk_component_list):
        trunk.paint_uniform_color(colors[i])
        pcd_list.append(trunk.get_pcd())  
    o3d.visualization.draw_geometries(pcd_list)


def o3dpc_to_rospc(o3dpc, frame_id=None, stamp=None):
    """
    This converts Open3D geometry into a ROS sensor_msgs/PointCloud2.
    """
    # bit operations
    BIT_MOVE_16 = 2 ** 16
    BIT_MOVE_8 = 2 ** 8

    cloud_npy = np.asarray(copy.deepcopy(o3dpc.points))
    is_color = len(o3dpc.colors) == len(o3dpc.points)

    n_points = len(cloud_npy[:, 0])
    if is_color:
        data = np.zeros(n_points, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)])
    else:
        data = np.zeros(n_points, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
    data['x'] = cloud_npy[:, 0]
    data['y'] = cloud_npy[:, 1]
    data['z'] = cloud_npy[:, 2]

    if is_color:
        rgb_npy = np.asarray(copy.deepcopy(o3dpc.colors))
        rgb_npy = np.floor(rgb_npy * 255)  # nx3 matrix
        rgb_npy = rgb_npy[:, 0] * BIT_MOVE_16 + rgb_npy[:, 1] * BIT_MOVE_8 + rgb_npy[:, 2]
        rgb_npy = rgb_npy.astype(np.uint32)
        data['rgb'] = rgb_npy

    rospc = ros_numpy.msgify(PointCloud2, data)
    if frame_id is not None:
        rospc.header.frame_id = frame_id

    if stamp is None:
        rospc.header.stamp = rospy.Time.now()
    else:
        rospc.header.stamp = stamp
    rospc.height = 1
    rospc.width = n_points
    rospc.fields = []
    rospc.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
    rospc.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
    rospc.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))

    if is_color:
        rospc.fields.append(PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1))
        rospc.point_step = 16
    else:
        rospc.point_step = 12

    rospc.is_bigendian = False
    rospc.row_step = rospc.point_step * n_points
    rospc.is_dense = True
    return rospc
