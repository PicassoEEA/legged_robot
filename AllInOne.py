#!/usr/bin/python3
# coding=utf8
# Date:2023/02/18
# Author:Xiteng_Yao
# 
# The code was inspired by the source codes of the PuppyPi product. 
# The code is completely rewritten to integrate all the functions of the robot, including the functions of the camera and LiDar

import math
import time
import rospy
import threading
from threading import RLock, Timer
import numpy as np
from std_msgs.msg import Float32
import geometry_msgs.msg as geo_msg
import sensor_msgs.msg as sensor_msg
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Empty, Trigger, TriggerRequest, TriggerResponse
from puppy_control.srv import SetInt64, SetInt64Request, SetInt64Response
from puppy_control.srv import SetFloat64List, SetFloat64ListRequest, SetFloat64ListResponse

import sys
import cv2
from std_srvs.srv import *
from sensor_msgs.msg import Image
from sensor.msg import Led
from object_tracking.srv import *
from puppy_control.msg import Velocity, Pose, Gait

from puppy_pi import Misc, apriltag


# Shared data array
shared_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

ROS_NODE_NAME = 'combined_control'

# Parameter for LiDAR
MAX_SCAN_ANGLE = 360  # degrees of LiDAR scan. If needed, minus a certain number from this

# Variables for AprilTag detections
tag_id = None
haved_detect = False

__isRunning = False

org_image_sub_ed = False

action_finish = True

lock = RLock()

# Parameters for AprilTag camera
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
camera_intrinsic = np.matrix([[619.063979, 0, 302.560920],
                              [0, 613.745352, 237.714934],
                              [0, 0, 1]])

times = 0

# Function for AprilTag detection
def apriltagDetect(img):
    global times
    global coordinate

    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    # Detect AprilTags in the grayscale image
    detections = detector.detect(gray, return_image=False)
    if len(detections) != 0:
        for detection in detections:
            # Compute the pose of the detected AprilTag
            M, e0, e1 = detector.detection_pose(detection, [camera_intrinsic.item(0, 0), camera_intrinsic.item(1, 1),
                                                            camera_intrinsic.item(0, 2), camera_intrinsic.item(1, 2)],
                                                0.033)

            # Extract the position of the detected AprilTag
            P = M[:3, :4]
            coordinate = np.matmul(P, np.array([[0], [0], [0], [1]])).flatten()
            print('coordinate = ', coordinate)

            # Check if the detected AprilTag belongs to the 'tag36h11' family
            if str(detection.tag_family, encoding='utf-8') == 'tag36h11':
                tag_id = str(detection.tag_id)  # get tag_id
                return tag_id
            else:
                return None
    else:
        times += 1
        if times >= 3:
            coordinate = None
        return None

# Function for analyzing AprilTag detections in the image
def tagAnalysis(img):
    global tag_id
    global haved_detect

    if not __isRunning:
        return img

    tag_id = apriltagDetect(img)  # AprilTag detection
    print("detecting tag", tag_id)
    if tag_id is not None and not haved_detect:
        haved_detect = True
    cv2.putText(img, tag_id, (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 3)
    return img

class Combined_Control:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        rospy.on_shutdown(self.cleanup)
        self.name = name
        self.running_mode = 1  # 1: LiDAR obstacle avoidance mode, 2: LiDAR guard mode
        self.threshold = 0.3  # Distance threshold in meters
        self.scan_angle = math.radians(90)  # Forward scan angle in radians
        self.speed = 0.12  # Obstacle avoidance mode speed in meters
        self.last_act = 0
        self.timestamp = 0
        self.lock = threading.RLock()
        self.lidar_sub = None
        self.velocity_pub = rospy.Publisher('/cmd_vel_nav', geo_msg.Twist, queue_size=1)
        self.velocity_pub.publish(geo_msg.Twist())
        self.lidar_sub = rospy.Subscriber('/scan', sensor_msg.LaserScan, self.lidar_callback) 
        self.buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
        self.buzzer_pub.publish(0.1)
        
    def lidar_callback(self, lidar_data: sensor_msg.LaserScan):
        twist = geo_msg.Twist()  # Initialize twist message
        max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)  # Calculate maximum index for left and right ranges
        left_ranges = lidar_data.ranges[:max_index]  # Get left ranges
        right_ranges = lidar_data.ranges[::-1][:max_index]  # Get right ranges

        with self.lock:  # Ensure thread safety
            angle = self.scan_angle / 2  # Calculate half of the scan angle
            angle_index = int(angle / lidar_data.angle_increment + 0.50)  # Calculate angle index
            
            # Restrict left and right ranges to the angle index
            left_ranges, right_ranges = left_ranges[:angle_index], right_ranges[:angle_index]

            # If running in obstacle avoidance mode (mode 1) and the current time is greater than the timestamp
            if self.running_mode == 1 and self.timestamp <= time.time():
                # Find the minimum distance and corresponding angle for left and right ranges
                min_index_left, min_index_right = np.nanargmin(np.array(left_ranges)), np.nanargmin(np.array(right_ranges))
                min_dist_left, min_dist_right = left_ranges[min_index_left], right_ranges[min_index_right]
                angle_left = lidar_data.angle_increment * min_index_left
                angle_right = lidar_data.angle_increment * min_index_right

                shared_data[0] = 1

                # If an obstacle is detected on the left side
                if min_dist_left <= self.threshold and min_dist_right > self.threshold:
                    twist.linear.x = self.speed / 6  # Reduce linear speed
                    max_angle = math.radians(90)
                    w = self.speed * 3
                    twist.angular.z = -w  # Rotate right
                    self.timestamp = time.time() + (max_angle / w / 2)  # Update timestamp

                # If obstacles are detected on both sides
                elif min_dist_left <= self.threshold and min_dist_right <= self.threshold:
                    twist.linear.x = self.speed / 6  # Reduce linear speed
                    w = self.speed * 3
                    twist.angular.z = w  # Rotate left
                    self.timestamp = time.time() + (math.radians(30) / w / 2)  # Update timestamp

                # If an obstacle is detected on the right side
                elif min_dist_left > self.threshold and min_dist_right <= self.threshold:
                    twist.linear.x = self.speed / 6  # Reduce linear speed
                    max_angle = math.radians(90)
                    w = self.speed * 3
                    twist.angular.z = w  # Rotate left
                    self.timestamp = time.time() + (max_angle / w / 2)  # Update timestamp

                # If no obstacles are detected
                else:
                    self.last_act = 0
                    twist.linear.x = self.speed  # Maintain linear speed
                    twist.angular.z = 0  # Do not rotate

                    shared_data[0] = 0

                shared_data[1] = twist

            # If running in guard mode (mode 2)
            elif self.running_mode == 2:
                # Find the minimum distance for left and right ranges
                min_index_left, min_index_right = np.nanargmin(np.array(left_ranges)), np.nanargmin(np.array(right_ranges))
                min_dist_left, min_dist_right = left_ranges[min_index_left], right_ranges[min_index_right]
                
                # If an obstacle is detected on either side
                if min_dist_left <= self.threshold or min_dist_right <= self.threshold:
                    self.buzzer_pub.publish(0.1)  # Publish to buzzer to alert about detected obstacle

    def cleanup(self):
        # This function is called when the node is shutting down
        # Unregister the lidar subscriber
        self.lidar_sub.unregister()
        # Publish a zero velocity command to stop the robot
        self.velocity_pub.publish(geo_msg.Twist())
        print('is_shutdown')

is_shutdown = False  # Global flag to check if the node is shutting down

PuppyMove = {'x':0, 'y':0, 'yaw_rate':0}  # Dictionary to store the robot's movement commands

color_range_list = {}  # Dictionary to store color range lists

__isRunning = False  # Flag to check if the node is running
__target_color = ''  # Variable to store the target color
org_image_sub_ed = False  # Flag to check if the original image is subscribed

line_centerx = -1  # Center x-coordinate of the detected line
img_centerx = 320  # Center x-coordinate of the image

# Region of interest (ROI) and corresponding weights for line detection
roi = [
        (240, 280,  0, 640, 0.1),
        (320, 360,  0, 640, 0.2),
        (400, 440,  0, 640, 0.7)
       ]
roi = [
        (120, 140,  0, 320, 0.1),
        (160, 180,  0, 320, 0.2),
        (200, 220,  0, 320, 0.7)
       ]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]
roi_h_list = [roi_h1, roi_h2, roi_h3]  # Heights of the ROIs

# Dictionary containing RGB values for different colors
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}
draw_color = range_rgb["black"]  # Default draw color is black

lock = RLock()  # Reentrant lock for thread synchronization


# Function to reset variables
def reset():
    global draw_color
    global __target_color
    global color_range_list
    global line_centerx
    with lock:
        # Turn off RGB display
        turn_off_rgb()
        # Reset target color
        __target_color = 'None'
        # Reset line center x-coordinate
        line_centerx = -1
        # Reset draw color to black
        draw_color = range_rgb["black"]
        # Update color range list from ROS parameter server
        color_range_list = rospy.get_param('/lab_config_manager/color_range_list')

        # Reset robot's movement commands
        PuppyMove['x'] = 0
        PuppyMove['yaw_rate'] = math.radians(0)
        PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])

# Function to initialize robot's position
def initMove(delay=True):
    global GaitConfig
    # Set initial movement commands
    PuppyMove['x'] = 0
    PuppyMove['yaw_rate'] = math.radians(0)
    PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])
    rospy.sleep(0.2)
    # Call go_home service to move robot to initial position
    rospy.ServiceProxy('/puppy_control/go_home', Empty)()

    # Set initial robot pose parameters
    PuppyPose["height"] = 0
    PuppyPose["pitch"] = -0.15

    # Publish initial robot pose
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                         height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500)

    rospy.sleep(0.2)
    # Publish initial gait configuration
    PuppyGaitConfigPub.publish(overlap_time=GaitConfig['overlap_time'], swing_time=GaitConfig['swing_time'],
                               clearance_time=GaitConfig['clearance_time'], z_clearance=GaitConfig['z_clearance'])

    with lock:
        pass
    if delay:
        rospy.sleep(0.5)
        
# Function to turn off RGB display
def turn_off_rgb():
    led = Led()
    led.index = 0
    led.rgb.r = 0
    led.rgb.g = 0
    led.rgb.b = 0
    rgb_pub.publish(led)
    rospy.sleep(0.005)
    led.index = 1
    rgb_pub.publish(led)

# Function to turn on RGB display with specified color
def turn_on_rgb(color):
    led = Led()
    led.index = 0
    led.rgb.r = range_rgb[color][2]
    led.rgb.g = range_rgb[color][1]
    led.rgb.b = range_rgb[color][0]
    rgb_pub.publish(led)
    rospy.sleep(0.005)
    led.index = 1
    rgb_pub.publish(led)
    rospy.sleep(0.1)

# Function called during app initialization
def init():
    print("Visual patrol Init")
    # Initialize robot's position and movement commands
    initMove(True)
    # Reset global variables related to target color, line center, and color range list
    reset()

def move():
    global PuppyMove
    global draw_color

    global line_centerx
    rospy.sleep(1)
    while True:
        if __isRunning:
            
            # If there's an obstacle detected or no line detected, execute this loop
            while line_centerx == -1 or shared_data[0] == 1:
                # If an obstacle is detected, execute this loop
                while shared_data[0] == 1:
                    # node.velocity_pub.publish(shared_data[1])
                    print("obstacle detected, avoiding")
                    PuppyMove['x'] = 0
                    if shared_data[1].linear.x > 0:
                        PuppyMove['yaw_rate'] = 15
                    else:
                        PuppyMove['yaw_rate'] = -15
                        
                    PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])
                    shared_data[2] += shared_data[1].angular.z
                    
                # If no line is detected, execute this loop
                while line_centerx == -1:
                    PuppyMove['x'] = 0
                    
                    if shared_data[2] > 0:
                        PuppyMove['yaw_rate'] = math.radians(-15)
                    else:
                        PuppyMove['yaw_rate'] = math.radians(15)
                        
                    PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])
                    print("no line detected, waiting")
                    rospy.sleep(0.1)
            
            # Reset the shared data
            shared_data[2] = 0

            # If a line is detected, adjust the robot's movement based on the line's position
            if line_centerx != -1:
                if abs(line_centerx - img_centerx) <= 50:
                    PuppyMove['x'] = 10
                    PuppyMove['yaw_rate'] = math.radians(0)
                elif line_centerx - img_centerx > 50:
                    PuppyMove['x'] = 8
                    PuppyMove['yaw_rate'] = math.radians(-15) # was 15 before
                    
                elif line_centerx - img_centerx < -50:
                    PuppyMove['x'] = 8
                    PuppyMove['yaw_rate'] = math.radians(15) # was 15 before
                
                PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])
            else:
                PuppyMove['x'] = 0
                PuppyMove['yaw_rate'] = math.radians(0)
                PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])
            time.sleep(0.001)
        else:
            time.sleep(0.001)
        if is_shutdown:break

# Start the move function in a separate daemon thread
th = threading.Thread(target=move, daemon=True)
# th.start()

# Find the contour with the largest area
# Parameter is a list of contours to compare
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # Iterate through all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:  # Only consider the largest contour valid if its area is greater than 50, to filter out noise
                area_max_contour = c

    return area_max_contour, contour_area_max  # Return the largest contour

# Main logic is here
# Need to look into and adjust
def lineCenterAnalysis(img):
    global draw_color, line_centerx
    size = (320, 240)
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img

    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB color space

    centroid_x_sum = 0
    weight_sum = 0
    center_ = []
    n = 0

    # Split the image into three parts (top, middle, and bottom) for faster and more accurate processing
    for r in roi:
        roi_h = roi_h_list[n]
        n += 1       
        if n <= 2:
            continue
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # Convert image to LAB color space

        for i in color_range_list:
            if i in __target_color:
                detect_color = i
                
                frame_mask = cv2.inRange(frame_lab,
                                             (color_range_list[detect_color]['min'][0],
                                              color_range_list[detect_color]['min'][1],
                                              color_range_list[detect_color]['min'][2]),
                                             (color_range_list[detect_color]['max'][0],
                                              color_range_list[detect_color]['max'][1],
                                              color_range_list[detect_color]['max'][2]))  # Perform bitwise operation on the original image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Apply opening operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # Apply closing operation

        if __target_color == '' or __target_color == 'None' or __target_color == None:
            line_centerx = -1
            return img

        cnts = cv2.findContours(closed , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2] # Find all contours
        cnt_large, area = getAreaMaxContour(cnts) # Find the contour with the largest area
        if cnt_large is not None: # If the contour is not empty
            rect = cv2.minAreaRect(cnt_large) # Minimum bounding rectangle
            box = np.int0(cv2.boxPoints(rect)) # Four vertices of the minimum bounding rectangle
            for i in range(4):
                box[i, 1] = box[i, 1] + (n - 1)*roi_h + roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
            for i in range(4):
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))

            cv2.drawContours(img, [box], -1, (0, 0, 255, 255), 2)  # Draw the rectangle formed by the four points

            # Get the diagonal points of the rectangle
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2  # Calculate the center point
            cv2.circle(img, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)  # Draw the center point

            center_.append([center_x, center_y])
            # Calculate the weighted sum of the three center points (top, middle, bottom)
            centroid_x_sum += center_x * r[4]
            weight_sum += r[4]

    if weight_sum != 0:
        # Calculate the final center point
        cv2.circle(img, (line_centerx, int(center_y)), 10, (0, 255, 255), -1)  # Draw the center point
        line_centerx = int(centroid_x_sum / weight_sum)
        print('line_centerx', line_centerx)
    else:
        line_centerx = -1

    return img
def image_callback(ros_image):
    global lock
    
    # Convert custom image message to an image
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)
    frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    # Create temporary and result frames for further processing
    frame_temp = frame.copy()
    frame_result = frame
    frame_result2 = frame
    
    # Process the frame using lineCenterAnalysis and tagAnalysis functions
    # Display the processed frames
    with lock:
        if __isRunning:
            frame_result = lineCenterAnalysis(frame)
            cv2.imshow('Frame', frame_result)
            key = cv2.waitKey(1)
            
            frame_result2 = tagAnalysis(frame_temp)
            cv2.imshow('image', frame_result2)
            cv2.waitKey(1)

def enter_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed

    # Log that the visual patrol has been entered
    rospy.loginfo("enter visual patrol")
    
    # Initialize the required variables and subscribe to the image topic
    with lock:
        init()
        if not org_image_sub_ed:
            org_image_sub_ed = True
            image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
            
    return [True, 'enter']
def start_running():
    global lock
    global __isRunning

    # Log the start of running
    rospy.loginfo("start running")
    with lock:
        __isRunning = True
    
def cleanup():
    global is_shutdown
    is_shutdown = True
    # Stop the puppy and log the shutdown status
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('is_shutdown')
    
    
if __name__ == "__main__":
    node = Combined_Control('combined_control_lidar')

    # Handle cleanup on shutdown
    rospy.on_shutdown(cleanup)
    
    # Get PuppyPose parameters and GaitConfig parameters
    PP = rospy.get_param('/puppy_control/PuppyPose')
    PuppyPose = PP['LookDown_20deg'].copy()
    GaitConfig = {'overlap_time':0.1, 'swing_time':0.15, 'clearance_time':0.0, 'z_clearance':3}
    
    # Initialize publishers and subscribers
    image_pub = rospy.Publisher('/%s/image_result'%ROS_NODE_NAME, Image, queue_size=1)
    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    rospy.sleep(0.2)

    th.start()
    debug = True
    
    # Enter the visual patrol function, start running, and set target color for debugging
    if debug:
        enter_func(1)
        start_running()
        __target_color = 'black'

    # Main loop
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
