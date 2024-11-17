# #!/usr/bin/env python3
#
# import rospy
# from sensor_msgs.msg import LaserScan, Image
# from nav_msgs.msg import Odometry, OccupancyGrid
# import numpy as np
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
#
#
# class RaceMapNode:
#     def __init__(self):
#         rospy.init_node('race_map_node')
#
#         # Subscribers
#         self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
#         self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
#         self.camera_sub = rospy.Subscriber('/camera/image', Image, self.camera_callback)
#
#         # Publishers
#         self.global_map_pub = rospy.Publisher('/global_map', OccupancyGrid, queue_size=10)
#
#         # Data storage
#         self.lidar_data = None
#         self.odom_data = None
#         self.camera_image = None
#         self.bridge = CvBridge()
#
#         # Map properties
#         self.map_resolution = 0.02  # Meters per cell
#         self.map_size = 500  # Map is 1000x1000 cells (100m x 100m area)
#         self.global_map = np.zeros((self.map_size, self.map_size), dtype=np.uint8)
#         self.origin_x = -5  # Map origin in meters
#         self.origin_y = -5
#
#         rospy.loginfo("RaceMapNode initialized")
#
#     def lidar_callback(self, msg):
#         self.lidar_data = msg
#
#     def odom_callback(self, msg):
#         self.odom_data = msg.pose.pose
#
#     def camera_callback(self, msg):
#         try:
#             self.camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr(f"Camera conversion error: {e}")
#
#     def transform_to_global(self, x, y):
#         """Transform local coordinates to global map coordinates."""
#         if self.odom_data is None:
#             rospy.logwarn("Odometry unavailable, using local coordinates")
#             return x, y
#
#         robot_x = self.odom_data.position.x
#         robot_y = self.odom_data.position.y
#         theta = 2 * np.arctan2(self.odom_data.orientation.z, self.odom_data.orientation.w)
#
#         # Apply the rotation and translation to transform local to global
#         x_global = x * np.cos(theta) - y * np.sin(theta) + robot_x
#         y_global = x * np.sin(theta) + y * np.cos(theta) + robot_y
#         return x_global, y_global
#
#     def process_lidar(self):
#         """Map LiDAR data into global coordinates."""
#         if self.lidar_data is None:
#             rospy.logwarn("No LiDAR data available")
#             return None
#
#         ranges = np.array(self.lidar_data.ranges)
#         angles = np.linspace(self.lidar_data.angle_min, self.lidar_data.angle_max, len(ranges))
#         lidar_map = np.zeros_like(self.global_map)
#
#         for i, r in enumerate(ranges):
#             if np.isinf(r) or np.isnan(r) or r > 30.0:  # Extend range to fit larger map
#                 continue
#
#             # Local Cartesian coordinates
#             x_local = r * np.cos(angles[i])
#             y_local = r * np.sin(angles[i])
#
#             # Transform to global coordinates
#             x_global, y_global = self.transform_to_global(x_local, y_local)
#
#             # Map to grid indices
#             grid_x = int((x_global - self.origin_x) / self.map_resolution)
#             grid_y = int((y_global - self.origin_y) / self.map_resolution)
#
#             # Ensure the points are within the map boundaries
#             if 0 <= grid_x < self.map_size and 0 <= grid_y < self.map_size:
#                 lidar_map[grid_x, grid_y] = 255
#
#         return lidar_map
#
#     def update_global_map(self, lidar_map):
#         """Update global map with LiDAR data."""
#         if lidar_map is not None:
#             self.global_map = np.maximum(self.global_map, lidar_map)
#
#     def publish_global_map(self):
#         """Publish the global map."""
#         grid_msg = OccupancyGrid()
#         grid_msg.header.stamp = rospy.Time.now()
#         grid_msg.header.frame_id = "map"
#         grid_msg.info.resolution = self.map_resolution
#         grid_msg.info.width = self.map_size
#         grid_msg.info.height = self.map_size
#         grid_msg.info.origin.position.x = self.origin_x
#         grid_msg.info.origin.position.y = self.origin_y
#         grid_msg.info.origin.orientation.w = 1.0
#         grid_msg.data = (self.global_map / 255.0 * 100).astype(np.int8).flatten().tolist()
#         self.global_map_pub.publish(grid_msg)
#
#     def run(self):
#         """Main loop."""
#         rate = rospy.Rate(10)
#         while not rospy.is_shutdown():
#             lidar_map = self.process_lidar()
#             self.update_global_map(lidar_map)
#             self.publish_global_map()
#
#             # Debugging visualization
#             cv2.imshow("Global Map", self.global_map)
#             cv2.waitKey(1)
#             rate.sleep()
#
#
# if __name__ == '__main__':
#     try:
#         node = RaceMapNode()
#         node.run()
#     except rospy.ROSInterruptException:
#         pass

