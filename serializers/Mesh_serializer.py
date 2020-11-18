"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
import sensor_msgs.point_cloud2 as pc2
import open3d
import numpy as np
from pcl_msgs.msg import PolygonMesh, Vertices
from Base_Serializer import BaseSerializer
import ctypes, struct


class MeshSerializer(BaseSerializer):
    def __init__(self, topic_name, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT Mesh SERIALIZER")
        super(MeshSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ".obj"

        rospy.Subscriber(topic_name, PolygonMesh, self.callback)

    def callback(self, mesh_msg):
        # initialize objects
        source_np = []
        source_color = []
        mesh_obj = open3d.geometry.TriangleMesh()

        # for each point, make numpy array
        for p in pc2.read_points(mesh_msg.cloud, skip_nans=True):
            source_np.append([p[0], p[1], p[2]])
            test = p[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f', test)
            i = struct.unpack('>l', s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            source_color.append([r / 255, g / 255, b / 255])
            # print(source_color)

        # assign numpy array to Open3D object
        source_np = np.array(source_np)
        source_color = np.array(source_color)
        mesh_obj.vertices = open3d.utility.Vector3dVector(source_np)
        mesh_obj.vertex_colors = open3d.utility.Vector3dVector(source_color)

        # for each triangle, create a numpy array
        source_tris = []
        for v in mesh_msg.polygons:
            source_tris.append([v.vertices[0], v.vertices[1], v.vertices[2]])

        # assign the array of vertices to the Open3d object
        source_tris = np.array(source_tris)
        mesh_obj.triangles = open3d.utility.Vector3iVector(source_tris)

        # mesh_smp = mesh_obj.simplify_quadric_decimation(
        #     target_number_of_triangles=7000) #int(source_tris.shape[0] / 2))
        # print(f'Original mesh has {len(mesh_obj.vertices)} vertices and {len(mesh_obj.triangles)} triangles')
        # print(f'Simplified mesh has {len(mesh_smp.vertices)} vertices and {len(mesh_smp.triangles)} triangles')
        # open3d.visualization.draw_geometries([mesh_smp])

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        open3d.io.write_triangle_mesh(file_name, mesh_obj)

        # increment
        self.counter += 1

