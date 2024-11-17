#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import tf

class LineFollower:
    def __init__(self):
        # Initialize the node
        rospy.init_node("line_follower", anonymous=True)

        # Subscribe to the camera feed
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.camera_callback)

        # Publisher for robot velocity
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Publisher for line markers
        self.marker_pub = rospy.Publisher("/line_markers", Marker, queue_size=10)

        # OpenCV bridge
        self.bridge = CvBridge()

        # Transform listener
        self.tf_listener = tf.TransformListener()

        # Velocity command
        self.cmd = Twist()

        # Accumulate points for lines
        self.yellow_points = []
        self.white_points = []

    def camera_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process the image to detect lines
            self.process_image(frame)
        except Exception as e:
            rospy.logerr("Error processing image: %s", e)

    def process_image(self, frame):
        height, width, _ = frame.shape

        # Focus only on the bottom quarter of the image
        bottom_half = frame[int(height * 0.75):, :]

        # Perspective transformation
        src_points = np.float32([[0, height * 0.75], [width, height * 0.75], [0, height], [width, height]])
        dst_points = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_bottom = cv2.warpPerspective(frame, matrix, (width, height))

        # Convert to HSV
        hsv = cv2.cvtColor(warped_bottom, cv2.COLOR_BGR2HSV)

        # Yellow and white masks
        yellow_mask = cv2.inRange(hsv, (20, 100, 100), (30, 255, 255))
        yellow_mask[:, width // 2:] = 0

        white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))
        white_mask[:, : width // 2] = 0

        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        edges = cv2.Canny(combined_mask, 30, 100)

        # Zoom into edges
        zoomed_edges = cv2.resize(edges, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_LINEAR)
        zoomed_height, zoomed_width = zoomed_edges.shape

        # Find contours
        contours, _ = cv2.findContours(zoomed_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print(f"Number of contours detected: {len(contours)}")

        left_centers = []
        right_centers = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 50:  # Adjust area threshold
                moments = cv2.moments(contour)
                if moments["m00"] > 0:
                    cx = int(moments["m10"] / moments["m00"])
                    cy = int(moments["m01"] / moments["m00"])
                    print(f"Contour center: ({cx}, {cy}), Width/2: {zoomed_width // 2}")
                    if cx < zoomed_width // 2:
                        print("Classified as Left (Yellow)")
                        left_centers.append((cx, cy))
                    else:
                        print("Classified as Right (White)")
                        right_centers.append((cx, cy))
                    cv2.circle(zoomed_edges, (cx, cy), 5, (0, 255, 0), -1)  # Draw detected centers

        # Calculate centroids for left and right sides
        yellow_center = (
            int(sum(c[0] for c in left_centers) / len(left_centers)),
            int(sum(c[1] for c in left_centers) / len(left_centers)),
        ) if left_centers else None

        white_center = (
            int(sum(c[0] for c in right_centers) / len(right_centers)),
            int(sum(c[1] for c in right_centers) / len(right_centers)),
        ) if right_centers else None

        # Debug: Show calculated centers
        print(f"Yellow Center: {yellow_center}, White Center: {white_center}")

        # Calculate the centerline (midpoint) and adjust the robot's trajectory
        if yellow_center and white_center:
            midpoint = (
                (yellow_center[0] + white_center[0]) // 2,
                (yellow_center[1] + white_center[1]) // 2,
            )
            cv2.circle(zoomed_edges, midpoint, 10, (0, 0, 255), -1)  # Red dot for midpoint

            # Calculate error (difference between midpoint and center of the frame)
            frame_center_x = zoomed_width // 2
            error = midpoint[0] - frame_center_x
            print(f"Midpoint: {midpoint}, Frame Center: {frame_center_x}, Error: {error}")

            # Control logic: adjust angular velocity based on error
            self.cmd.linear.x = 0.2  # Constant forward speed
            self.cmd.angular.z = -float(error) / 50  # Proportional control for turning
            self.cmd_vel_pub.publish(self.cmd)

            # Publish the detected lines to SLAM map
            self.publish_lines_to_map(yellow_center, white_center)

        else:
            # Stop the robot if centroids are not detected
            print("Stopping robot: unable to detect both lines.")
            self.stop_robot()

        # Draw vertical line for context
        cv2.line(zoomed_edges, (zoomed_width // 2, 0), (zoomed_width // 2, zoomed_height), (255, 0, 0), 2)

        # Display the processed image
        cv2.imshow("Line Follower", zoomed_edges)
        cv2.waitKey(1)

    def image_point_to_3d(self, image_point):
        """Convert 2D image point to 3D camera coordinates."""
        fx, fy = 600.0, 600.0  # Adjust based on calibration
        cx, cy = 320.0, 240.0  # Adjust based on image center
        z = -0.5 # Assume Z=1 meter
        x = (image_point[1] - cx) / fx
        y = (image_point[0] - cy) / fy
        print(f"Image Point: {image_point}, 3D Camera Coordinates: {(x * z, y * z, z)}")
        return (x * z, y * z, z)

    def transform_point_to_map(self, point, trans, rot):
        """Transform a point from the camera frame to the map frame."""
        point_camera = np.array([point[0], point[1], point[2], 1])  # Homogeneous coordinates
        transform_matrix = tf.transformations.quaternion_matrix(rot)  # Get rotation matrix
        transform_matrix[:3, 3] = trans  # Add translation component

        print(f"Transform Matrix:\n{transform_matrix}")
        print(f"Point in Camera Frame: {point}")

        point_map = np.dot(transform_matrix, point_camera)  # Transform to map frame
        print(f"Point in Map Frame: {point_map[:3]}")
        return tuple(point_map[:3])  # Return (x, y, z)
    def publish_line_strip(self, points, ns, color):
        """Publish a line strip as a visualization marker."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Thickness of the line
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        # Convert points to geometry_msgs/Point
        marker.points = []
        for p in points:
            point = Marker().pose.position  # geometry_msgs/Point
            point.x, point.y, point.z = p[0], p[1], p[2]
            marker.points.append(point)

        self.marker_pub.publish(marker)
    def publish_lines_to_map(self, yellow_center, white_center):
        """Publish yellow and white lines as line strips in the SLAM map."""
        try:
            self.tf_listener.waitForTransform("map", "camera_link", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform("map", "camera_link", rospy.Time(0))

            if yellow_center:
                yellow_point_camera = self.image_point_to_3d(yellow_center)
                yellow_point_map = self.transform_point_to_map(yellow_point_camera, trans, rot)
                self.yellow_points.append(yellow_point_map)  # Accumulate points
                self.publish_line_strip(self.yellow_points, "yellow_line", (1.0, 1.0, 0.0))
            else:
                print("No Yellow Center Detected")

            if white_center:
                white_point_camera = self.image_point_to_3d(white_center)
                white_point_map = self.transform_point_to_map(white_point_camera, trans, rot)
                self.white_points.append(white_point_map)  # Accumulate points
                self.publish_line_strip(self.white_points, "white_line", (1.0, 1.0, 1.0))
            else:
                print("No White Center Detected")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF Error: {e}")

    def publish_marker(self, point, ns, color):
        """Publish a visualization marker."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


    def stop_robot(self):
        """Stop the robot."""
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        line_follower = LineFollower()
        line_follower.run()
    except rospy.ROSInterruptException:
        pass
