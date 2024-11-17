#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Define states
FOLLOWING_LINE = 0
AVOIDING_OBSTACLE = 1
SEARCHING_FOR_LINE = 2
SEARCHING_FOR_SIDE_OBSTACLE = 3
GAP_FINDING = 4

# Define the proportions for left and right ROIs
LEFT_ROI_PROPORTION = 0.5
RIGHT_ROI_PROPORTION = 0.5

# Global Variables
lidar_data = None
angle_min = None
angle_max = None
angle_increment = None
current_linear_speed = 0.3
current_state = FOLLOWING_LINE
integral = 0.0
previous_error = 0.0

# Constants
MAX_LINEAR_SPEED = 0.6
MIN_LINEAR_SPEED = 0.1
MAX_ANGULAR_SPEED = 4  # Maximum angular speed for line following
TURN_SPEED = 0.55        # Angular speed for turning
OBSTACLE_DISTANCE_THRESHOLD_FRONT = 0.55  # Meters
OBSTACLE_DISTANCE_THRESHOLD_SIDE = 0.3  # Meters
OBSTACLE_CLEAR_DISTANCE = 0.5            # For hysteresis
LOOKAHEAD_DISTANCE = 50
DISPLAY_ROIS = True
CAMERA_HORIZONTAL_FOV = 60

# PID constants
Kp = 0.04      # Proportional gain
Ki = 0.0003    # Integral gain
Kd = 0.04      # Derivative gain

# Colors for visualization
YELLOW_LOWER = np.array([20, 100, 100])
YELLOW_UPPER = np.array([30, 255, 255])
WHITE_LOWER = np.array([0, 0, 220])
WHITE_UPPER = np.array([180, 25, 255])  # Adjusted for better white detection

def lidar_callback(data):
    global lidar_data, angle_min, angle_max, angle_increment
    lidar_data = data.ranges
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment

def detect_widest_opening(lidar_data, front_angle_range=(325, 360, 0, 45)):
    """
    Detects the direction with the widest open space in the specified front angle range.
    Returns the angle with the maximum available distance and the distance itself.
    """
    if lidar_data is None:
        rospy.logwarn("No LIDAR data available.")
        return None, float('inf')

    num_measurements = len(lidar_data)
    angles = np.linspace(angle_min, angle_max, num_measurements)
    angles_deg = np.degrees(angles) % 360

    # Front left and right ranges based on the specified angles
    front_indices = np.where(
        ((angles_deg >= front_angle_range[0]) & (angles_deg <= front_angle_range[1])) |
        ((angles_deg >= front_angle_range[2]) & (angles_deg <= front_angle_range[3]))
    )[0]
    front_distances = [lidar_data[i] for i in front_indices if not np.isinf(lidar_data[i]) and not np.isnan(lidar_data[i])]

    if not front_distances:
        return None, float('inf')

    max_distance = max(front_distances)
    max_index = front_distances.index(max_distance)
    angle = angles_deg[front_indices[max_index]]

    return angle, max_distance
def refined_obstacle_avoidance(lidar_data, threshold_front=OBSTACLE_DISTANCE_THRESHOLD_FRONT):
    global current_state
    obstacle_front, angle_front, distance_front = detect_obstacle_front(lidar_data)
    if obstacle_front:
        rospy.loginfo("Obstacle detected ahead, avoiding obstacle.")
        angle, distance = detect_widest_opening(lidar_data)

        if distance > threshold_front:  # Move towards open direction
            angular_z = -TURN_SPEED if angle > 180 else TURN_SPEED
            rospy.loginfo(f"Turning towards open angle: {angle} with distance: {distance:.2f}")
        else:  # Narrow space, just turn left to avoid
            angular_z = TURN_SPEED
        return angular_z, 0.0  # Make sure both return values are single numbers
    else:
        # Transition to SEARCHING_FOR_SIDE_OBSTACLE after clearing the front
        rospy.loginfo("Front obstacle cleared. Searching for side obstacles.")
        current_state = SEARCHING_FOR_SIDE_OBSTACLE
        return 0.0, current_linear_speed

def robot_behavior_controller(cv_image, velocity_publisher):
    global current_state, lidar_data, integral, previous_error, current_linear_speed
    height, width = cv_image.shape[:2]
    vel_msg = Twist()
    angular_z = 0.0
    linear_x = current_linear_speed  # Default linear speed

    # Display mask for debugging
    mask_visual = np.zeros((height, width, 3), dtype=np.uint8)

    # Detect line masks and separate lines
    full_mask, left_width, right_start = detect_line_masks(cv_image)
    left_lines, right_lines = detect_lines_separately(full_mask, width, left_width, right_start)
    left_fit, right_fit = average_lines(left_lines), average_lines(right_lines)

    # Visualization of detected lines (optional)
    mask_visual = cv2.cvtColor(full_mask, cv2.COLOR_GRAY2BGR)
    if DISPLAY_ROIS:
        if left_lines:
            for line in left_lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(mask_visual, (x1, y1 + height // 2), (x2, y2 + height // 2), (255, 0, 0), 2)
        if right_lines:
            for line in right_lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(mask_visual, (x1, y1 + height // 2), (x2, y2 + height // 2), (0, 255, 0), 2)

    # Check for a near-horizontal line indicating a potential intersection or stop
    horizontal_detected = False
    for line in left_lines + right_lines:
        for x1, y1, x2, y2 in line:
            slope = (y2 - y1) / (x2 - x1) if x2 - x1 != 0 else float('inf')
            if abs(slope) < 0.5:
                horizontal_detected = True
                break
        if horizontal_detected:
            break

    if horizontal_detected:
        rospy.loginfo_throttle(5, "Horizontal line detected. Handling intersection or stop line.")
        linear_x = 0.0
        angular_z = 0.0
        vel_msg.linear.x = linear_x
        vel_msg.angular.z = angular_z
        velocity_publisher.publish(vel_msg)
        return

    # Detect obstacles using LIDAR data
    obstacle_front, _, _ = detect_obstacle_front(lidar_data)
    side_obstacles = detect_obstacle_side(lidar_data)
    obstacle_right = side_obstacles["right"]
    obstacle_left = side_obstacles["left"]

    # State Machine for robot behavior
    if current_state == FOLLOWING_LINE:
        if obstacle_front:
            rospy.loginfo("Obstacle detected in front. Switching to AVOIDING_OBSTACLE state.")
            current_state = AVOIDING_OBSTACLE
            integral = 0.0
            previous_error = 0.0
        elif obstacle_right[0] or obstacle_left[0]:  # Detect obstacles on either side
            rospy.loginfo("Obstacle detected on the side.")
            return
        else:
            # Line following behavior
            if left_fit is not None or right_fit is not None:
                target_x = find_lookahead_point(left_fit, right_fit, height, width)
                center_x = width // 2
                error = center_x - target_x
                integral += error
                derivative = error - previous_error
                angular_z = Kp * error + Ki * integral + Kd * derivative
                previous_error = error
                linear_x = min(current_linear_speed + 0.02, MAX_LINEAR_SPEED)
            else:
                rospy.loginfo_throttle(5, "No lines detected. Switching to SEARCHING_FOR_LINE state.")
                current_state = SEARCHING_FOR_LINE
                integral = 0.0
                previous_error = 0.0

    elif current_state == AVOIDING_OBSTACLE:
        # Refined obstacle avoidance
        angular_z, linear_x = refined_obstacle_avoidance(lidar_data)
        vel_msg.angular.z = angular_z
        vel_msg.linear.x = linear_x
        velocity_publisher.publish(vel_msg)
        return

    elif current_state == SEARCHING_FOR_SIDE_OBSTACLE:
        # Continue driving straight until a side obstacle is close
        if (obstacle_right[0] and obstacle_right[2] < OBSTACLE_DISTANCE_THRESHOLD_SIDE) :
            rospy.loginfo("Close side obstacle detected. Switching to SEARCHING_FOR_LINE.")
            vel_msg.angular.z = 1.2
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(1)
            current_state = SEARCHING_FOR_LINE


        if (obstacle_left[0] and obstacle_left[2] < OBSTACLE_DISTANCE_THRESHOLD_SIDE):
            rospy.loginfo("Close side obstacle detected. Switching back to SEARCHING_FOR_LINE.")
            vel_msg.angular.z = -1.2
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(1)
            current_state = SEARCHING_FOR_LINE
        linear_x = current_linear_speed
        angular_z = 0.0

    elif current_state == SEARCHING_FOR_LINE:
        rospy.loginfo("SEARCHING_FOR_LINE.")
        if left_fit is not None or right_fit is not None:
            rospy.loginfo("Line found. Switching back to FOLLOWING_LINE state.")
            current_state = FOLLOWING_LINE
            integral = 0.0
            previous_error = 0.0
        else:
            rospy.loginfo_throttle(5, "Searching for line.")
            angular_z = search_for_line_behavior()
            linear_x = MIN_LINEAR_SPEED

    # Ensure angular_z is within limits
    angular_z = max(min(angular_z, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)
    vel_msg.linear.x = linear_x
    vel_msg.angular.z = angular_z
    velocity_publisher.publish(vel_msg)

    # Display the masked image with detected lines if enabled
    if DISPLAY_ROIS:
        cv2.imshow("Masked Image with Detected Lines", mask_visual)
        cv2.waitKey(1)



def line_following_behavior(cv_image):
    global previous_error, integral, current_linear_speed
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask_yellow = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
    mask_white = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
    mask = cv2.bitwise_or(mask_yellow, mask_white)

    M = cv2.moments(mask)
    if M['m00'] == 0:
        rospy.logwarn_throttle(5, "No line detected. Stopping.")
        return 0.0, 0.0
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    height, width = mask.shape
    center_x = width // 2

    error = center_x - cx

    P = Kp * error

    integral += error
    I = Ki * integral

    derivative = error - previous_error
    D = Kd * derivative

    angular_z = P + I + D

    previous_error = error

    linear_x = current_linear_speed

    if DISPLAY_ROIS:
        cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)  # Red dot at centroid
        cv2.line(cv_image, (center_x, height), (cx, cy), (255, 0, 0), 2)  # Blue line from center to centroid
        cv2.imshow("Line Following", cv_image)
        cv2.waitKey(1)

    rospy.loginfo_throttle(1, f"Line Following - Error: {error}, Angular_z: {angular_z}, Linear_x: {linear_x}")
    return angular_z, linear_x

def detect_obstacle_front(lidar_data, threshold=OBSTACLE_DISTANCE_THRESHOLD_FRONT):
    if lidar_data is None:
        rospy.logwarn("LIDAR data is None.")
        return False, 0.0, float('inf')

    num_measurements = len(lidar_data)
    angles = np.linspace(angle_min, angle_max, num_measurements)
    angles_deg = np.degrees(angles) % 360  # [0, 360)
    front_indices = np.where((angles_deg <= 5) | (angles_deg >= 355))[0]
    front_distances = [lidar_data[i] for i in front_indices if not np.isinf(lidar_data[i]) and not np.isnan(lidar_data[i])]

    if not front_distances:
        return False, 0.0, float('inf')

    min_distance = min(front_distances)
    min_index = front_distances.index(min_distance)
    angle = angles_deg[front_indices[min_index]]

    if min_distance < threshold:
        rospy.loginfo_throttle(1, f"Obstacle detected in front at angle: {angle:.2f} degrees, distance: {min_distance:.2f} meters")
        return True, angle, min_distance
    else:
        return False, 0.0, min_distance

def detect_obstacle_side(lidar_data, threshold=OBSTACLE_DISTANCE_THRESHOLD_SIDE):
    if lidar_data is None:
        rospy.logwarn("LIDAR data is None.")
        return {"right": (False, 0.0, float('inf')), "left": (False, 0.0, float('inf'))}

    num_measurements = len(lidar_data)
    angles = np.linspace(angle_min, angle_max, num_measurements)
    angles_deg = np.degrees(angles) % 360  # Normalize to [0, 360) degrees

    # Right side angles (85-95 degrees)
    side_indices_right = np.where((angles_deg >= 95) & (angles_deg <= 105))[0]
    side_distances_right = [lidar_data[i] for i in side_indices_right if not np.isinf(lidar_data[i]) and not np.isnan(lidar_data[i])]

    # Left side angles (265-275 degrees)
    side_indices_left = np.where((angles_deg >= 275) & (angles_deg <= 285))[0]
    side_distances_left = [lidar_data[i] for i in side_indices_left if not np.isinf(lidar_data[i]) and not np.isnan(lidar_data[i])]

    # Process right side obstacle
    if side_distances_right:
        min_distance_right = min(side_distances_right)
        min_index_right = side_distances_right.index(min_distance_right)
        angle_right = angles_deg[side_indices_right[min_index_right]]
        obstacle_right = min_distance_right < threshold
    else:
        obstacle_right, angle_right, min_distance_right = False, 0.0, float('inf')

    # Process left side obstacle
    if side_distances_left:
        min_distance_left = min(side_distances_left)
        min_index_left = side_distances_left.index(min_distance_left)
        angle_left = angles_deg[side_indices_left[min_index_left]]
        obstacle_left = min_distance_left < threshold
    else:
        obstacle_left, angle_left, min_distance_left = False, 0.0, float('inf')

    # Log detection on either side
    if obstacle_right:
        rospy.loginfo_throttle(1, f"Obstacle detected on the right side at angle: {angle_right:.2f} degrees, distance: {min_distance_right:.2f} meters")
    if obstacle_left:
        rospy.loginfo_throttle(1, f"Obstacle detected on the left side at angle: {angle_left:.2f} degrees, distance: {min_distance_left:.2f} meters")

    # Return obstacle information for both sides
    return {
        "right": (obstacle_right, angle_right, min_distance_right),
        "left": (obstacle_left, angle_left, min_distance_left)
    }


def detect_obstacle(lidar_data, threshold=OBSTACLE_DISTANCE_THRESHOLD_FRONT):
    obstacle_front, angle_front, distance_front = detect_obstacle_front(lidar_data, threshold)
    obstacle_side, angle_side, distance_side = detect_obstacle_side(lidar_data, threshold=OBSTACLE_DISTANCE_THRESHOLD_SIDE)
    return (obstacle_front, angle_front, distance_front), (obstacle_side, angle_side, distance_side)

def obstacle_avoidance_behavior():
    angular_z = TURN_SPEED
    linear_x = 0.0
    return angular_z, linear_x

def search_for_line_behavior():
    search_angular_speed = 0
    linear_x = 0
    rospy.loginfo_throttle(5, "Searching for line. Rotating in place.")
    return search_angular_speed  # This should be a single numeric value


def detect_line_masks(cv_image):
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    height, width = hsv.shape[:2]
    half_height = height // 2

    roi = hsv[half_height:, :]

    left_width = int(width * LEFT_ROI_PROPORTION)
    right_start = int(width * (1 - RIGHT_ROI_PROPORTION))

    left_roi = roi[:, :left_width]
    right_roi = roi[:, right_start:]

    mask_yellow = cv2.inRange(left_roi, YELLOW_LOWER, YELLOW_UPPER)
    mask_white = cv2.inRange(right_roi, WHITE_LOWER, WHITE_UPPER)

    full_mask = np.zeros_like(hsv[:, :, 0])

    full_mask[half_height:, :left_width] = mask_yellow
    full_mask[half_height:, right_start:] = mask_white

    return full_mask, left_width, right_start

def detect_lines_separately(full_mask, width, left_width, right_start):

    edges = cv2.Canny(full_mask, 50, 50)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=15, minLineLength=20, maxLineGap=10)
    left_lines, right_lines = [], []

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x2 - x1 == 0:
                    continue  # Avoid division by zero
                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) < 0.5:
                    continue  # Ignore near-horizontal lines
                if x1 < left_width and x2 < left_width:
                    left_lines.append(line)
                elif x1 > right_start and x2 > right_start:
                    right_lines.append(line)

    return left_lines, right_lines

def average_lines(lines):
    if not lines:
        return None

    x_coords, y_coords = [], []
    for line in lines:
        for x1, y1, x2, y2 in line:
            x_coords.extend([x1, x2])
            y_coords.extend([y1, y2])

    if len(x_coords) == 0:
        return None

    slope, intercept = np.polyfit(x_coords, y_coords, 1)
    return slope, intercept

def make_line_points(height, slope, intercept):
    y1 = height
    y2 = int(height * 0.6)
    if slope == 0:
        slope = 0.1
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return (x1, y1), (x2, y2)

def find_lookahead_point(left_fit, right_fit, height, width, lookahead_distance=LOOKAHEAD_DISTANCE):
    y = height - lookahead_distance
    x_left = int((y - left_fit[1]) / left_fit[0]) if left_fit else None
    x_right = int((y - right_fit[1]) / right_fit[0]) if right_fit else None

    if x_left is not None and x_right is not None:
        target = int((x_left + x_right) / 2)
    elif x_left is not None:
        target = x_left + 80
        target = min(target, width - 1)
    elif x_right is not None:
        target = x_right - 80
        target = max(target, 0)
    else:
        target = width // 2
    target = max(0, min(target, width - 1))
    return target

def camera_callback(data, args):
    velocity_publisher, bridge = args
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    robot_behavior_controller(cv_image, velocity_publisher)

def shutdown_hook(velocity_publisher):
    rospy.loginfo("Shutting down Line Following Node. Stopping the robot.")
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)
    cv2.destroyAllWindows()

def main():
    rospy.init_node('line_following_with_obstacle_avoidance', anonymous=True)

    rospy.loginfo("Starting Line Following with Obstacle Avoidance Node.")

    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    bridge = CvBridge()

    rospy.Subscriber('/camera/image', Image, camera_callback, (velocity_publisher, bridge))
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.on_shutdown(lambda: shutdown_hook(velocity_publisher))

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception. Shutting down node.")
