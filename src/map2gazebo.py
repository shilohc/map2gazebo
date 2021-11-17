#!/usr/bin/env python

import cv2
import numpy as np
import trimesh
from matplotlib.tri import Triangulation

import rospy
from nav_msgs.msg import OccupancyGrid

class MapConverter(object):
    def __init__(self, map_topic, threshold=1, height=2.0):
        self.test_map_pub = rospy.Publisher(
                "test_map", OccupancyGrid, latch=True, queue_size=1)
        rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
        self.threshold = threshold
        self.height = height
        # Probably there's some way to get trimesh logs to point to ROS
        # logs, but I don't know it.  Uncomment the below if something
        # goes wrong with trimesh to get the logs to print to stdout.
        #trimesh.util.attach_to_log()

    def map_callback(self, map_msg):
        rospy.loginfo("Received map")
        map_dims = (map_msg.info.height, map_msg.info.width)
        map_array = np.array(map_msg.data).reshape(map_dims)

        # set all -1 (unknown) values to 0 (unoccupied)
        map_array[map_array < 0] = 0
        contours = self.get_occupied_regions(map_array)
        meshes = [self.contour_to_mesh(c, map_msg.info) for c in contours]

        corners = list(np.vstack(contours))
        corners = [c[0] for c in corners]
        self.publish_test_map(corners, map_msg.info, map_msg.header)
        mesh = trimesh.util.concatenate(meshes)

        # Export as STL or DAE
        mesh_type = rospy.get_param("~mesh_type", "stl")
        export_dir = rospy.get_param("~export_dir")
        if mesh_type == "stl":
            with open(export_dir + "/map.stl", 'w') as f:
                mesh.export(f, "stl")
            rospy.loginfo("Exported STL.  You can shut down this node now")
        elif mesh_type == "dae":
            with open(export_dir + "/map.dae", 'w') as f:
                f.write(trimesh.exchange.dae.export_collada(mesh))
            rospy.loginfo("Exported DAE.  You can shut down this node now")

    def publish_test_map(self, points, metadata, map_header):
        """
        For testing purposes, publishes a map highlighting certain points.
        points is a list of tuples (x, y) in the map's coordinate system.
        """
        test_map = np.zeros((metadata.height, metadata.width))
        for x, y in points:
            test_map[y, x] = 100
        test_map_msg = OccupancyGrid()
        test_map_msg.header = map_header
        test_map_msg.header.stamp = rospy.Time.now()
        test_map_msg.info = metadata
        test_map_msg.data = list(np.ravel(test_map))
        self.test_map_pub.publish(test_map_msg)

    def get_occupied_regions(self, map_array):
        """
        Get occupied regions of map
        """
        map_array = map_array.astype(np.uint8)
        _, thresh_map = cv2.threshold(
                map_array, self.threshold, 100, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(
                thresh_map, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        # Using cv2.RETR_CCOMP classifies external contours at top level of
        # hierarchy and interior contours at second level.  
        # If the whole space is enclosed by walls RETR_EXTERNAL will exclude
        # all interior obstacles e.g. furniture.
        # https://docs.opencv.org/trunk/d9/d8b/tutorial_py_contours_hierarchy.html
        hierarchy = hierarchy[0]
        corner_idxs = [i for i in range(len(contours)) if hierarchy[i][3] == -1]
        return [contours[i] for i in corner_idxs]

    def contour_to_mesh(self, contour, metadata):
        height = np.array([0, 0, self.height])
        s3 = 3**0.5 / 3.
        meshes = []
        for point in contour:
            x, y = point[0]
            vertices = []
            new_vertices = [
                    coords_to_loc((x, y), metadata),
                    coords_to_loc((x, y+1), metadata),
                    coords_to_loc((x+1, y), metadata),
                    coords_to_loc((x+1, y+1), metadata)]
            vertices.extend(new_vertices)
            vertices.extend([v + height for v in new_vertices])
            faces = [[0, 2, 4],
                     [4, 2, 6],
                     [1, 2, 0],
                     [3, 2, 1],
                     [5, 0, 4],
                     [1, 0, 5],
                     [3, 7, 2],
                     [7, 6, 2],
                     [7, 4, 6],
                     [5, 4, 7],
                     [1, 5, 3],
                     [7, 3, 5]]
            mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
            if not mesh.is_volume:
                rospy.logdebug("Fixing mesh normals")
                mesh.fix_normals()
            meshes.append(mesh)
        mesh = trimesh.util.concatenate(meshes)
        mesh.remove_duplicate_faces()
        # mesh will still have internal faces.  Would be better to get
        # all duplicate faces and remove both of them, since duplicate faces
        # are guaranteed to be internal faces
        return mesh

def coords_to_loc(coords, metadata):
    x, y = coords
    loc_x = x * metadata.resolution + metadata.origin.position.x
    loc_y = y * metadata.resolution + metadata.origin.position.y
    # TODO: transform (x*res, y*res, 0.0) by Pose map_metadata.origin
    # instead of assuming origin is at z=0 with no rotation wrt map frame
    return np.array([loc_x, loc_y, 0.0])

if __name__ == "__main__":
    rospy.init_node("map2gazebo")
    map_topic = rospy.get_param("~map_topic", "map")
    occupied_thresh = rospy.get_param("~occupied_thresh", 1)
    box_height = rospy.get_param("~box_height", 2.0)
    converter = MapConverter(map_topic,
            threshold=occupied_thresh, height=box_height)
    rospy.loginfo("map2gazebo running")
    rospy.spin()
