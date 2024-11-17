#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def compress_fov(cv_image):
    # Define the region for a bird's-eye view of the ground lines
    # The points are chosen to simulate a top-down view to focus on ground lines
    image_height, image_width = cv_image.shape[:2]
    bottom_left = [int(0.1 * image_width), image_height]
    bottom_right = [int(0.9 * image_width), image_height]
    top_left = [int(0.3 * image_width), int(0.6 * image_height)]
    top_right = [int(0.7 * image_width), int(0.6 * image_height)]

    # Create an array of points for the perspective transform
    pts1 = np.float32([bottom_left, bottom_right, top_left, top_right])

    # The size to which you want to map the ROI
    width, height = image_width, int(0.7 * image_height)
    pts2 = np.float32([[0, height], [width, height], [0, 0], [width, 0]])

    # Compute the perspective transform matrix
    matrix = cv2.getPerspectiveTransform(pts1, pts2)

    # Apply the perspective transform to get a warped image
    birdseye_image = cv2.warpPerspective(cv_image, matrix, (width, height))

    return birdseye_image


def camera_test():
    rospy.init_node('camera_test_node', anonymous=True)
    bridge = CvBridge()

    def camera_callback(data):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

            # Apply bird's-eye view compression for parking spot detection
            compressed_image = compress_fov(cv_image)

            # Display the original and compressed images
            cv2.imshow("Original Camera Feed", cv_image)
            cv2.imshow("Compressed Bird's-Eye View", compressed_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)

    rospy.Subscriber('/camera/image', Image, camera_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        camera_test()
    except rospy.ROSInterruptException:
        pass
