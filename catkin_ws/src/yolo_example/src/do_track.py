#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import math

# Initialize global variables
integral = 0.0
previous_error = 0.0
lidar_data = None
state = "BETWEEN_LINES"
rotation_direction = None
obstacle_detected = False
obstacle_avoidance_cooldown = None
cooldown_duration = 1
move_forward_duration = 1.5
forward_start_time = None
align_threshold = 0.2
max_speed = 0.1
min_speed = 0.01
error_threshold = 50
sharp_turn_threshold = 0.28
stop_sign_cooldown_time = 2
tunnel_sign_cooldown_time = 5
ignore_tunnel_sign_duration = 8
last_stop_sign_detected_time = 0
last_tunnel_sign_detected_time = 0
ignore_stop_sign = False
ignore_tunnel_sign = False
ignore_stop_sign_duration = 8  # Duration to ignore the stop sign after detection
current_time = time.time()
# Cooldown for arrow detection
arrow_detection_cooldown = 8  # Seconds
last_arrow_detection_time = 0

# Define HSV ranges for colored objects (excluding black and white, and excluding stop sign)
colored_object_hsv_ranges = [
    (np.array([20, 50, 50]), np.array([30, 255, 255])),  # Yellow
]

# General blue HSV range for both arrow and parking signs
blue_lower = np.array([90, 50, 70])
blue_upper = np.array([130, 255, 255])

# Adjusted HSV ranges for white color
white_lower = np.array([0, 0, 240])
white_upper = np.array([180, 20, 255])

# Define HSV ranges for stop sign (red color)
stop_sign_hsv_ranges = [
    (np.array([0, 70, 50]), np.array([10, 255, 255])),    # Red range 1
    (np.array([160, 70, 50]), np.array([180, 255, 255]))  # Red range 2
]

# Track whether the robot has already responded to the stop sign
detected_stop_sign = False
last_p_sign_detection_time = None

def compress_fov(cv_image):
    image_height, image_width = cv_image.shape[:2]
    bottom_left = [int(0.1 * image_width), image_height]
    bottom_right = [int(0.9 * image_width), image_height]
    top_left = [int(0.3 * image_width), int(0.6 * image_height)]
    top_right = [int(0.7 * image_width), int(0.6 * image_height)]
    pts1 = np.float32([bottom_left, bottom_right, top_left, top_right])
    width, height = image_width, int(0.7 * image_height)
    pts2 = np.float32([[0, height], [width, height], [0, 0], [width, 0]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    compressed_image = cv2.warpPerspective(cv_image, matrix, (width, height))

    return compressed_image

def lidar_callback(data):
    global lidar_data
    lidar_data = data.ranges

def set_fixed_avoidance_direction():
    global rotation_direction
    if rotation_direction is None:
        rotation_direction = "right" if state == "BETWEEN_LINES" else "left"

def reset_fixed_avoidance_direction():
    global rotation_direction
    rotation_direction = None

def detect_p_shape(blue_mask):
    global last_p_sign_detection_time
    current_time = time.time()

    # Implement cooldown
    if last_p_sign_detection_time is not None and (current_time - last_p_sign_detection_time) < 60:
        rospy.loginfo("P sign detection is on cooldown.")
        return False

    # Find contours in the blue mask
    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        rospy.loginfo("No contours found for 'P' detection.")
        return False

    for contour in contours:
        if cv2.contourArea(contour) < 350:
            continue

        # Get the convex hull and defects
        hull = cv2.convexHull(contour, returnPoints=False)
        if len(hull) < 4:
            continue

        defects = cv2.convexityDefects(contour, hull)
        if defects is not None and len(defects) > 0:
            # rospy.loginfo(f"Number of convexity defects = {len(defects)}")
            if len(defects) == 3 or len(defects) == 2 or len(defects) == 1:
                rospy.loginfo("Parking sign detected based on convexity defects.")
                last_p_sign_detection_time = current_time  # Update cooldown time
                return True

    # rospy.loginfo("No convexity defect pattern matching 'P' found.")
    return False

def detect_arrow_direction(blue_mask):
    # Find contours in the blue mask
    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # rospy.loginfo(f"Number of contours found for arrow detection: {len(contours)}")

    if not contours:
        # rospy.loginfo("No contours found for arrow detection.")
        return None

    # Assume the largest contour is the arrow
    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)
    # rospy.loginfo(f"Arrow Detection: Arrow contour area = {area}")

    # Adjust contour area threshold if needed
    if area < 500:  # Adjust the area threshold to filter small objects
        # rospy.loginfo("Contour area too small for arrow detection.")
        return None

    # Approximate the contour to simplify the shape
    epsilon = 0.02 * cv2.arcLength(largest_contour, True)  # Reduce epsilon to retain more points
    approx = cv2.approxPolyDP(largest_contour, epsilon, True)
    rospy.loginfo(f"Arrow Detection: Number of points in approximated contour = {len(approx)}")

    # Log each point to help debug
    for point in approx:
        rospy.loginfo(f"Point: {point[0]}")

    # Determine if the tip is on the left or right half of the contour's bounding box
    x, y, w, h = cv2.boundingRect(largest_contour)
    min_angle = 180
    tip_point = None

    for i in range(len(approx)):
        prev_point = approx[i - 1][0]
        current_point = approx[i][0]
        next_point = approx[(i + 1) % len(approx)][0]
        v1 = prev_point - current_point
        v2 = next_point - current_point
        denom = (np.linalg.norm(v1) * np.linalg.norm(v2))

        if denom == 0:
            continue
        cos_angle = np.clip(np.dot(v1, v2) / denom, -1.0, 1.0)
        angle = math.degrees(math.acos(cos_angle))

        rospy.loginfo(f"Angle at point {i}: {angle}")

        if angle < min_angle:
            min_angle = angle
            tip_point = current_point

    if tip_point is None:
        rospy.loginfo("No valid tip point found for arrow detection.")
        return None

    rospy.loginfo(f"Arrow Detection: Tip point at x = {tip_point[0]}, Bounding box center x = {x + w / 2}")

    if tip_point[0] < x + w / 2:
        return "left_arrow"
    else:
        return "right_arrow"

def detect_stop_sign(cv_image):
    global detected_stop_sign, last_stop_sign_detected_time, ignore_stop_sign
    current_time = time.time()
    # Cooldown mechanism to avoid repeated stop sign detection
    if ignore_stop_sign:
        if current_time - last_stop_sign_detected_time < ignore_stop_sign_duration:
            rospy.loginfo("Stop sign detection is on cooldown.")
            return False
        else:
            ignore_stop_sign = False  # Reset ignore flag after duration

    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    for lower, upper in stop_sign_hsv_ranges:
        mask = cv2.inRange(hsv_image, lower, upper)
        count = cv2.countNonZero(mask)
        rospy.loginfo(f"Stop Sign Detection: Red pixel count = {count}")
        if count > 700 and not detected_stop_sign:
            rospy.loginfo("Stop sign detected.")
            detected_stop_sign = True  # Mark that the stop sign has been detected
            last_stop_sign_detected_time = current_time  # Update the last detection time
            ignore_stop_sign = True  # Set ignore flag to prevent repeated stops
            return True
    rospy.loginfo("No stop sign detected.")
    return False

def detect_tunnel_sign(cv_image):
    global detected_tunnel_sign, last_tunnel_sign_detected_time, ignore_tunnel_sign
    current_time = time.time()
    # Cooldown mechanism to avoid repeated tunnel sign detection
    if ignore_tunnel_sign:
        if current_time - last_tunnel_sign_detected_time < ignore_stop_sign_duration:
            rospy.loginfo("Tunnel sign detection is on cooldown.")
            return False
        else:
            ignore_tunnel_sign = False  # Reset ignore flag after duration

    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define HSV range for the red outer part of the tunnel sign
    red_lower_1 = np.array([0, 100, 100])
    red_upper_1 = np.array([10, 255, 255])
    red_lower_2 = np.array([160, 100, 100])
    red_upper_2 = np.array([180, 255, 255])

    # Create masks for the red color
    red_mask_1 = cv2.inRange(hsv_image, red_lower_1, red_upper_1)
    red_mask_2 = cv2.inRange(hsv_image, red_lower_2, red_upper_2)
    red_mask = red_mask_1 | red_mask_2

    # Define HSV range for the orange inner part of the tunnel sign
    orange_lower = np.array([10, 100, 100])
    orange_upper = np.array([25, 255, 255])
    orange_mask = cv2.inRange(hsv_image, orange_lower, orange_upper)

    # Find contours in the red mask to locate potential tunnel signs
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in red_contours:
        if 450 > cv2.contourArea(contour) > 350:  # Ensure the red area is large enough
            # Create a bounding box around the red contour
            x, y, w, h = cv2.boundingRect(contour)
            # Extract the region of interest (ROI) for the orange part
            roi = orange_mask[y:y+h, x:x+w]
            # Count the number of orange pixels in the ROI
            orange_count = cv2.countNonZero(roi)
            rospy.loginfo(f"Tunnel Sign Detection: Orange pixel count in red bounding box = {orange_count}")
            if 200 > orange_count > 100:  # Threshold for detecting the orange part inside the red part
                rospy.loginfo("Tunnel sign detected.")
                detected_tunnel_sign = True  # Mark that the tunnel sign has been detected
                last_tunnel_sign_detected_time = current_time  # Update the last detection time
                ignore_tunnel_sign = True  # Set ignore flag to prevent repeated detections
                return True

    rospy.loginfo("No tunnel sign detected.")
    return False


def process_sign(cv_image, velocity_publisher):
    global last_arrow_detection_time, state
    if state == "TUNNEL_MODE":
        rospy.loginfo("Tunnel mode active. Ignoring other signs.")
        return False

    if detect_stop_sign(cv_image):
        rospy.loginfo("STOP sign detected. Stopping robot.")
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        rospy.sleep(3)
        return True
    if detect_tunnel_sign(cv_image):
        rospy.loginfo("Tunnel sign detected. Stopping robot. Activating tunnel mode")
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        rospy.sleep(1)
        vel_msg.linear.x = 0.2
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        rospy.sleep(2)
        state = "TUNNEL_MODE"
        return True

    # Create mask for blue
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)

    # Log blue pixel count for debugging
    blue_pixel_count = cv2.countNonZero(blue_mask)
    # rospy.loginfo(f"Blue pixel count = {blue_pixel_count}")

    # Clean up the blue mask if needed
    kernel = np.ones((3, 3), np.uint8)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

    # Check if the shape is more like a 'P' or an arrow
    if detect_p_shape(blue_mask):
        rospy.loginfo("Parking sign detected. Switching to PARKING_MODE.")
        vel_msg = Twist()
        vel_msg.linear.x = 0.06
        vel_msg.angular.z = -0.4
        velocity_publisher.publish(vel_msg)
        rospy.sleep(3)
        state = "PARKING_MODE"
        return True
    elif current_time - last_arrow_detection_time > arrow_detection_cooldown:
        direction = detect_arrow_direction(blue_mask)
        if direction == "left_arrow":
            rospy.loginfo("Left arrow detected. Applying cooldown.")
            vel_msg = Twist()
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0)
            last_arrow_detection_time = current_time
            return True
        elif direction == "right_arrow":
            rospy.loginfo("Right arrow detected. Applying cooldown.")
            vel_msg = Twist()
            vel_msg.linear.x = 0.15
            vel_msg.angular.z = -0.25
            velocity_publisher.publish(vel_msg)
            rospy.sleep(2)
            last_arrow_detection_time = current_time
            return True

    rospy.loginfo("No relevant sign detected.")
    return False


def follow_reversed_yellow_line(cv_image, velocity_publisher):
    rospy.loginfo("State: FOLLOW_REVERSED_YELLOW_LINE")

    # Get the dimensions of the image
    height, width, _ = cv_image.shape

    # Focus only on the very bottom part of the image
    bottom_height = height // 4  # Bottom quarter of the image
    lower_image = cv_image[-bottom_height:, :]  # Only bottom part of the image

    # Convert to HSV for color detection
    hsv_image = cv2.cvtColor(lower_image, cv2.COLOR_BGR2HSV)

    # Define the right half of the bottom portion for yellow detection
    right_bottom = hsv_image[:, width // 2:]  # Bottom right half

    # Yellow detection in the right half
    yellow_lower = np.array([20, 100, 100])  # Adjusted based on calibration
    yellow_upper = np.array([30, 255, 255])
    yellow_mask = cv2.inRange(right_bottom, yellow_lower, yellow_upper)
    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Visualize yellow mask and contours for debugging
    # cv2.imshow("Yellow Mask", yellow_mask)
    # cv2.drawContours(right_bottom, yellow_contours, -1, (0, 255, 0), 3)
    # cv2.imshow("Yellow Contours", right_bottom)
    # cv2.waitKey(1)

    # Debug: Show number of yellow contours detected
    rospy.loginfo(f"Number of yellow contours detected: {len(yellow_contours)}")

    # Check if significant yellow is detected
    yellow_cx = None
    if yellow_contours:
        largest_yellow_contour = max(yellow_contours, key=cv2.contourArea)
        if cv2.contourArea(largest_yellow_contour) > 50:  # Reduced threshold
            M_yellow = cv2.moments(largest_yellow_contour)
            if M_yellow["m00"] > 0:
                yellow_cx = int(M_yellow["m10"] / M_yellow["m00"])

    vel_msg = Twist()

    # Parameters for speed and sharp turn handling
    max_speed = 0.1
    min_speed = 0.01
    error_threshold = 50  # Adjusted threshold
    sharp_turn_threshold = 0.35  # Adjust as needed
    width_factor = right_bottom.shape[1]

    # Initialize yellow_lost_time if not already
    global yellow_lost_time
    if 'yellow_lost_time' not in globals():
        yellow_lost_time = None
    search_timeout = 5  # seconds

    if yellow_cx is not None:
        rospy.loginfo("Yellow line detected. Following it to exit parking.")
        yellow_lost_time = None  # Reset timer when yellow line is found

        # Desired position is the center of right_bottom
        desired_position_yellow = width_factor // 2
        error = yellow_cx - desired_position_yellow

        # Basic proportional control
        k_p = 0.004
        angular_correction = -k_p * error

        # Speed adjustment based on error
        speed_factor = max(0, 1 - (abs(error) / error_threshold))
        vel_msg.linear.x = min_speed + (max_speed - min_speed) * speed_factor
        vel_msg.angular.z = angular_correction

        # Handle sharp turns more smoothly
        if abs(error) > sharp_turn_threshold * width_factor:
            rospy.loginfo("Sharp turn detected. Slowing down and turning more aggressively.")
            vel_msg.linear.x = min_speed  # Slow down
            vel_msg.angular.z *= 1.6  # Increase turning rate

        # Debugging the error and correction values
        rospy.loginfo(f"Error: {error}, Angular Correction: {angular_correction}")

        # Publish velocity command
        velocity_publisher.publish(vel_msg)

        # Continue following the yellow line
        return False  # Return False to indicate no state change

    else:
        # Yellow line not detected
        rospy.loginfo("Yellow line lost. Checking for white line.")

        # Start the timer if it's not already running
        if yellow_lost_time is None:
            yellow_lost_time = time.time()
        elif time.time() - yellow_lost_time > search_timeout:
            rospy.loginfo("Yellow line not found within timeout. Switching state.")
            return True  # Switch state after timeout

        # Define the same area for white detection
        right_bottom = hsv_image[:, width // 2:]  # Bottom right half

        # White detection in the same area
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([180, 25, 255])
        white_mask = cv2.inRange(right_bottom, white_lower, white_upper)
        white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Debug: Show number of white contours detected
        rospy.loginfo(f"Number of white contours detected: {len(white_contours)}")

        white_cx = None
        if white_contours:
            largest_white_contour = max(white_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_white_contour) > 50:
                M_white = cv2.moments(largest_white_contour)
                if M_white["m00"] > 0:
                    white_cx = int(M_white["m10"] / M_white["m00"])

        if white_cx is not None:
            # White line detected
            rospy.loginfo("White line detected. Switching state to BETWEEN_LINES.")
            vel_msg.linear.x = 0.1
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(2)  # Pause for 2 seconds before switching state

            return True  # Return True to indicate state change to BETWEEN_LINES

        else:
            # Neither yellow nor white line detected
            rospy.logwarn("No line detected. Adjusting to search for the yellow line.")
            vel_msg.linear.x = min_speed  # Slow down
            vel_msg.angular.z = 0.1  # Rotate gently to search for the line

            # Publish velocity command
            velocity_publisher.publish(vel_msg)

            return False  # Continue in the current state




def camera_callback(data, args):
    global integral, previous_error, lidar_data, state, rotation_direction, obstacle_detected, obstacle_avoidance_cooldown, forward_start_time, detected_stop_sign, last_stop_sign_detected_time

    velocity_publisher, bridge = args

    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)
        return


    # Process all signs (stop, parking, and arrow)
    if not process_sign(cv_image, velocity_publisher):
        # Rest of your line-following and obstacle avoidance logic
        # rospy.loginfo("No relevant sign detected. Proceeding with line-following and obstacle avoidance.")

        # Existing line-following code
        height, width, _ = cv_image.shape
        lower_image = cv_image[(3 * height) // 4:, :]
        hsv_image = cv2.cvtColor(lower_image, cv2.COLOR_BGR2HSV)
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        white_lower_line = np.array([0, 0, 240])
        white_upper_line = np.array([180, 20, 255])
        yellow_mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)
        white_mask_line = cv2.inRange(hsv_image, white_lower_line, white_upper_line)
        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        white_contours, _ = cv2.findContours(white_mask_line, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        yellow_detected_bottom_left = any(cv2.contourArea(contour) > 250 for contour in yellow_contours)
        white_detected_bottom_right = any(cv2.contourArea(contour) > 250 for contour in white_contours)

        yellow_cx, white_cx = None, None
        if yellow_contours:
            largest_yellow_contour = max(yellow_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_yellow_contour) > 500:
                M_yellow = cv2.moments(largest_yellow_contour)
                if M_yellow["m00"] > 0:
                    yellow_cx = int(M_yellow["m10"] / M_yellow["m00"])

        if white_contours:
            largest_white_contour = max(white_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_white_contour) > 500:
                M_white = cv2.moments(largest_white_contour)
                if M_white["m00"] > 0:
                    white_cx = int(M_white["m10"] / M_white["m00"])

        vel_msg = Twist()
        front_ranges = lidar_data[350:] + lidar_data[:10] if lidar_data else []

        right_ranges = lidar_data[290:330] if lidar_data else []  # LiDAR range to the right side of the robot

        if state == "PARKING_MODE":
            rospy.loginfo("State: PARKING_MODE")
            # Apply bird's-eye view compression for parking spot detection
            compressed_image = compress_fov(cv_image)

            # Convert the compressed image to HSV
            hsv_compressed = cv2.cvtColor(compressed_image, cv2.COLOR_BGR2HSV)
            white_mask = cv2.inRange(hsv_compressed, white_lower, white_upper)

            # Detect white lines to determine potential parking spots on the right side
            white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            detected_spots = []

            for contour in white_contours:
                if cv2.contourArea(contour) > 5:  # Threshold to avoid small noise
                    x, y, w, h = cv2.boundingRect(contour)
                    # Focus on the right side of the compressed FOV
                    if x > (width // 2):
                        rospy.loginfo(f"Detected white line on the right side - Width: {w}, Height: {h}")

                        # Identify parking spot by checking the white line size and gap
                        if 25 < w < 40 and 25 < h < 50:
                            rospy.loginfo(f"Detected potential parking spot boundary. Height: {h}, Width: {w}")
                            detected_spots.append((x, y, w, h))

            # Continue to follow the yellow line while checking for parking spots
            if yellow_cx is not None:
                rospy.loginfo("PARKING_MODE: Following yellow line while checking for parking spots.")
                desired_position_yellow = int(0.2 * width)
                error = yellow_cx - desired_position_yellow
                integral += error * 0.1
                derivative = (error - previous_error) * 0.2
                previous_error = error
                speed_factor = max(0, 1 - (abs(error) / error_threshold))
                vel_msg.linear.x = min_speed + (max_speed - min_speed) * speed_factor
                k_p = 0.004
                k_i, k_d = 0.00001, 0.001
                vel_msg.angular.z = (-k_p * error) - (k_i * integral) - (k_d * derivative)

                # Handle sharp turns more smoothly
                if abs(error) > sharp_turn_threshold * width:
                    rospy.loginfo("Sharp turn detected. Slowing down and turning more aggressively.")
                    vel_msg.linear.x = min_speed  # Slow down
                    vel_msg.angular.z *= 1.6

                velocity_publisher.publish(vel_msg)
            rospy.loginfo(min(right_ranges))
            # Proceed with parking logic if two valid spots are found and right side is clear
            if len(detected_spots) >= 2:
                if min(right_ranges) > 0.5:  # Ensure right side is clear
                    # rospy.loginfo(
                    #     "Two empty parking spots detected on the right and right side is clear. Initiating parking maneuver.")
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = 0
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(1)

                    # Maneuver into parking spot (e.g., turn right and move forward)
                    vel_msg.angular.z = -0.5  # Turn right to align with parking spot
                    rospy.loginfo("Turning right to align with parking spot.")
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(2)  # Adjust timing as needed to align

                    vel_msg.angular.z = 0
                    vel_msg.linear.x = 0.11  # Move forward slowly into the spot
                    rospy.loginfo("Moving forward into the parking spot.")
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(3.3)  # Adjust timing as needed to park fully

                    vel_msg.angular.z = 0.5  # Adjust final alignment in the parking spot
                    vel_msg.linear.x = 0
                    rospy.loginfo("Final alignment adjustment in the parking spot.")
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(10)
                    vel_msg.angular.z = 0  # Adjust final alignment in the parking spot
                    vel_msg.linear.x = 0
                    rospy.loginfo("PARRRRKEEEEED.")
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(5)

                    rospy.loginfo("Successfully parked in the spot.")
                    state = "OUT_OF_PARKING"
                    return
                else:
                    rospy.logwarn("Right side is not clear according to LiDAR. Aborting parking maneuver.")
            else:
                rospy.logwarn("Not enough valid parking spots detected. Aborting parking maneuver.")
        elif state == "OUT_OF_PARKING":
            rospy.loginfo("State: OUT_OF_PARKING")
            vel_msg.angular.z = -0.5
            velocity_publisher.publish(vel_msg)
            rospy.sleep(4)
            vel_msg.linear.x = 0.1
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(2)
            vel_msg.angular.z = 0.5
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(4)
            vel_msg.angular.z = -0.03
            vel_msg.linear.x = 0.1
            velocity_publisher.publish(vel_msg)
            rospy.sleep(4)
            rospy.loginfo("Exiting the parking spot and resuming normal operation.")
            if follow_reversed_yellow_line(cv_image, velocity_publisher):
                state = "BETWEEN_LINES"
            return
        elif state == "WAIT":
            rospy.loginfo("State: WAIT")
            vel_msg.angular.z = 0.0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(1)
            rospy.loginfo("WAIITTTT.")

        if state == "TUNNEL_MODE":
            rospy.loginfo("State: TUNNEL_MODE")

            # Parameters for movement
            linear_velocity = 0.1  # Constant linear speed for moving forward
            angular_kp = 0.5  # Proportional control gain for turning
            max_angular_velocity = 0.6
            min_obstacle_distance = 0.4  # Minimum distance from an obstacle before taking action

            if lidar_data:
                # Convert LiDAR data to a numpy array for easy manipulation
                full_ranges = np.array(lidar_data)

                # Extract data for left front, right front, and straight ahead
                front_left_range = np.nanmin(full_ranges[30:60])  # Approximately 30-60 degrees for front left
                front_right_range = np.nanmin(full_ranges[290:330])  # Approximately 300-330 degrees for front right
                front_range = np.nanmin(
                    np.concatenate((full_ranges[345:], full_ranges[:15])))  # Approximately -10 to 10 degrees

                rospy.loginfo(
                    "Front Range: {:.2f}, Front Left: {:.2f}, Front Right: {:.2f}".format(front_range, front_left_range,
                                                                                          front_right_range))

                # Initialize Twist message for velocity
                vel_msg = Twist()

                if front_range < min_obstacle_distance:
                    rospy.loginfo("Obstacle detected directly in front. Choosing a direction to avoid.")

                    # Compare left and right front distances and decide the turn direction
                    if front_left_range > front_right_range:
                        rospy.loginfo("Turning left to avoid obstacle.")
                        vel_msg.angular.z = angular_kp
                    else:
                        rospy.loginfo("Turning right to avoid obstacle.")
                        vel_msg.angular.z = -angular_kp

                    # Limit the angular velocity to the maximum allowed value
                    vel_msg.angular.z = max(-max_angular_velocity, min(max_angular_velocity, vel_msg.angular.z))

                    # Move forward slowly while turning
                    vel_msg.linear.x = 0.05
                else:
                    # No obstacle directly ahead, move forward
                    rospy.loginfo("Path is clear, moving forward.")
                    vel_msg.angular.z = 0.0
                    vel_msg.linear.x = linear_velocity
                velocity_publisher.publish(vel_msg)

            # Improved Transition Logic to BETWEEN_LINES
            if yellow_detected_bottom_left and white_detected_bottom_right:
                # Ensure that both yellow and white lines have significant size before transitioning
                yellow_area = sum(
                    cv2.contourArea(contour) for contour in yellow_contours if cv2.contourArea(contour) > 250)
                white_area = sum(
                    cv2.contourArea(contour) for contour in white_contours if cv2.contourArea(contour) > 250)

                if yellow_area > 950 and white_area > 950:  # Tune this threshold based on your environment
                    rospy.loginfo(
                        "Both yellow (left) and white (right) lines detected with significant size. Transitioning to BETWEEN_LINES state.")
                    state = "BETWEEN_LINES"
                else:
                    rospy.loginfo("Yellow or white lines detected but not enough area. Continuing in TUNNEL_MODE.")
            else:
                rospy.loginfo("Yellow and white lines not consistently detected. Remaining in TUNNEL_MODE.")

            return





        elif state == "BETWEEN_LINES" and lidar_data:
            if min(front_ranges) < 0.4 and not obstacle_detected:
                set_fixed_avoidance_direction()
                rospy.loginfo(f"Obstacle detected directly ahead. Avoiding to the {rotation_direction}.")
                state = "AVOIDING_OBSTACLE"
                obstacle_detected = True
                obstacle_avoidance_cooldown = time.time() + cooldown_duration

        if state == "BETWEEN_LINES":
            rospy.loginfo("State: BETWEEN_LINES")
            if yellow_cx is not None and white_cx is not None:
                target_center = (yellow_cx + white_cx) // 2
                desired_center = width // 2
                error = target_center - desired_center
                integral += error * 0.1
                derivative = (error - previous_error) * 0.2
                previous_error = error
                speed_factor = max(0, 1 - (abs(error) / error_threshold))
                vel_msg.linear.x = min_speed + (max_speed - min_speed) * speed_factor
                k_p = 0.004
                k_i, k_d = 0.00001, 0.001
                vel_msg.angular.z = (-k_p * error) - (k_i * integral) - (k_d * derivative)

            elif yellow_cx is not None:
                rospy.loginfo("Only yellow line detected. Following yellow line.")
                desired_position_yellow = int(0.17 * width)
                error = yellow_cx - desired_position_yellow
                integral += error * 0.1
                derivative = (error - previous_error) * 0.2
                previous_error = error
                speed_factor = max(0, 1 - (abs(error) / error_threshold))
                vel_msg.linear.x = min_speed + (max_speed - min_speed) * speed_factor
                k_p = 0.004
                k_i, k_d = 0.00001, 0.001
                vel_msg.angular.z = (-k_p * error) - (k_i * integral) - (k_d * derivative)

                # Handle sharp turns more smoothly
                if abs(error) > sharp_turn_threshold * width:
                    rospy.loginfo("Sharp turn detected. Slowing down and turning more aggressively.")
                    vel_msg.linear.x = min_speed  # Slow down
                    vel_msg.angular.z *= 1.6

            elif white_cx is not None:
                rospy.loginfo("Only white line detected. Following white line.")
                desired_position_white = int(0.87 * width)
                error = white_cx - desired_position_white
                integral += error * 0.1
                derivative = (error - previous_error) * 0.2
                previous_error = error
                speed_factor = max(0, 1 - (abs(error) / error_threshold))
                vel_msg.linear.x = min_speed + (max_speed - min_speed) * speed_factor
                k_p = 0.004
                k_i, k_d = 0.00001, 0.001
                vel_msg.angular.z = (-k_p * error) - (k_i * integral) - (k_d * derivative)

                # Handle sharp turns more smoothly
                if abs(error) > sharp_turn_threshold * width:
                    rospy.loginfo("Sharp turn detected. Slowing down and turning more aggressively.")
                    vel_msg.linear.x = min_speed  # Slow down
                    vel_msg.angular.z *= 1.6

            else:
                rospy.logwarn("Both lines lost, slowing down and attempting to realign.")
                vel_msg.linear.x = min_speed
                vel_msg.angular.z = 0.2 if rotation_direction == "right" else -0.2

        elif state == "REALIGN_AFTER_ARROW":
            rospy.loginfo("State: REALIGN_AFTER_ARROW")
            if yellow_cx is not None:
                rospy.loginfo("Realigning with yellow line on the bottom left.")
                desired_position_yellow = int(0.17 * width)
                error = yellow_cx - desired_position_yellow
                integral += error * 0.1
                derivative = (error - previous_error) * 0.2
                previous_error = error
                speed_factor = max(0, 1 - (abs(error) / error_threshold))
                vel_msg.linear.x = min_speed + (max_speed - min_speed) * speed_factor
                k_p = 0.004
                k_i, k_d = 0.00001, 0.001
                vel_msg.angular.z = (-k_p * error) - (k_i * integral) - (k_d * derivative)

                # Switch back to normal line following if the robot has realigned properly
                if abs(error) < align_threshold * width:
                    rospy.loginfo("Successfully realigned with the yellow line. Switching to BETWEEN_LINES state.")
                    state = "BETWEEN_LINES"

            else:
                rospy.logwarn("Lost yellow line while realigning. Attempting to find it again.")
                vel_msg.angular.z = 0.2  # Rotate to search for the yellow line


        elif state == "AVOIDING_OBSTACLE":
            rospy.loginfo("State: AVOIDING_OBSTACLE")
            front_clearance = 0.6
            side_clearance = 0.3
            left_side = min(lidar_data[90:130]) if lidar_data else float('inf')
            right_side = min(lidar_data[230:270]) if lidar_data else float('inf')
            front_clear = min(front_ranges) > front_clearance

            if rotation_direction == "right":
                rospy.loginfo("Adjusting to avoid obstacle on the right side.")
                vel_msg.angular.z = 0.7
                vel_msg.linear.x = 0.05
            elif rotation_direction == "left":
                rospy.loginfo("Adjusting to avoid obstacle on the left side.")
                vel_msg.angular.z = -0.7
                vel_msg.linear.x = 0.05
            elif front_clear:
                rospy.loginfo("Path is clear, moving forward.")
                vel_msg.linear.x = 0.2
                vel_msg.angular.z = 0.0

            if front_clear and left_side > side_clearance and right_side > side_clearance:
                rospy.loginfo("Obstacle fully cleared. Continuing to navigate around.")
                state = "NAVIGATE_AROUND"
                forward_start_time = time.time()

        elif state == "NAVIGATE_AROUND":
            rospy.loginfo("State: NAVIGATE_AROUND")
            vel_msg.linear.x = 0.1
            side_clearance = 0.3
            left_side = min(lidar_data[90:130]) if lidar_data else float('inf')
            right_side = min(lidar_data[230:270]) if lidar_data else float('inf')
            front_clear = min(front_ranges) > 0.5

            if not front_clear:
                rospy.loginfo("Obstacle still ahead. Adjusting path.")
                state = "AVOIDING_OBSTACLE"
            elif left_side > side_clearance and right_side > side_clearance:
                rospy.loginfo("Obstacle fully cleared. Attempting to re-engage with lines.")
                state = "RE-ENGAGE_LINES"

        elif state == "RE-ENGAGE_LINES":
            rospy.loginfo("State: RE-ENGAGE_LINES")
            vel_msg.linear.x = 0.2
            vel_msg.angular.z = 0.7 if rotation_direction == "right" else -0.7
            if yellow_cx is not None or white_cx is not None:
                rospy.loginfo("Line detected. Switching back to BETWEEN_LINES.")
                state = "BETWEEN_LINES"
                reset_fixed_avoidance_direction()

        velocity_publisher.publish(vel_msg)

def main():
    rospy.init_node('turtlebot_between_lines_follower', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    bridge = CvBridge()

    rospy.Subscriber('/camera/image', Image, camera_callback, (velocity_publisher, bridge))
    rospy.Subscriber('/scan', LaserScan, lidar_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception. Shutting down node.")
        pass
