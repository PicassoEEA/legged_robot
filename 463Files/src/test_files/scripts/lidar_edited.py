#!/usr/bin/python3
# coding=utf8
# Date:2022/12/27
# Author:hiwonder, Xiteng Yao

import math
import time
import rospy
import threading
import numpy as np
from std_msgs.msg import Float32
import geometry_msgs.msg as geo_msg
import sensor_msgs.msg as sensor_msg

MAX_SCAN_ANGLE = 360 # LiDar scan angle, 360 degrees, minus the angles that cannot be scanned

class LidarController:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        rospy.on_shutdown(self.cleanup)
        self.name = name
        self.running_mode = 1 # 1：LiDar avoidance mode, 2：LiDar buzzer mode
        self.threshold = 0.25 # meters  distance threshold for object avoidance, 0.3 is recommended
        self.scan_angle = math.radians(90)  # radians  forward scan angle
        self.speed = 0.15 # speed of the robot in object avoidance mode m/s, 0.12 recommended
        self.last_act = 0
        self.timestamp = 0
        self.lock = threading.RLock()
        self.lidar_sub = None
        self.velocity_pub = rospy.Publisher('/cmd_vel_nav', geo_msg.Twist, queue_size=1)
        self.velocity_pub.publish(geo_msg.Twist())
        self.lidar_sub = rospy.Subscriber('/scan', sensor_msg.LaserScan, self.lidar_callback) 
        self.buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
        self.buzzer_pub.publish(0.1)
        
    def lidar_callback(self, lidar_data:sensor_msg.LaserScan):
        twist = geo_msg.Twist()
        max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
        left_ranges = lidar_data.ranges[:max_index]
        right_ranges = lidar_data.ranges[::-1][:max_index]
        
        with self.lock:
            #angle = math.atan(self.car_width / self.threshold)
            angle = self.scan_angle / 2
            angle_index = int(angle / lidar_data.angle_increment + 0.50)
            
            left_ranges, right_ranges = left_ranges[:angle_index], right_ranges[:angle_index]
            
            if self.running_mode == 1 and self.timestamp <= time.time():
                min_index_left, min_index_right = np.nanargmin(np.array(left_ranges)),  np.nanargmin(np.array(right_ranges))
                min_dist_left, min_dist_right = left_ranges[min_index_left], right_ranges[min_index_right]
                angle_left = lidar_data.angle_increment * min_index_left
                angle_right = lidar_data.angle_increment * min_index_right
                # print(min_dist_left, min_dist_right)
                
                # obstacle on the left
                if min_dist_left <= self.threshold and min_dist_right > self.threshold: 
                    twist.linear.x = self.speed / 6
                    max_angle = math.radians(90)
                    w = self.speed * 3
                    twist.angular.z = -w
                    self.velocity_pub.publish(twist)
                    self.timestamp = time.time() + (max_angle / w / 2)
                
                # obstacle on the left and right    
                elif min_dist_left <= self.threshold and min_dist_right <= self.threshold: 
                    twist.linear.x = self.speed / 6
                    w = self.speed * 3
                    twist.angular.z = w
                    # if self.last_act != 0:
                    #     twist.angular.z = -w
                    # else:
                    #     self.last_act = 1
                    self.velocity_pub.publish(twist)
                    self.timestamp = time.time() + (math.radians(30) / w / 2)
            
                # obstacle on the right    
                elif min_dist_left > self.threshold and min_dist_right <= self.threshold: 
                    twist.linear.x = self.speed / 6
                    max_angle = math.radians(90)
                    w = self.speed * 3
                    twist.angular.z = w
                    # if self.last_act != 0:
                    #     twist.angular.z = -w
                    # else:
                    #     self.last_act = 2
                    self.velocity_pub.publish(twist)
                    self.timestamp = time.time() + (max_angle / w /2)
                    
                # no obstacle    
                else: 
                    self.last_act = 0
                    twist.linear.x = self.speed
                    twist.angular.z = 0
                    self.velocity_pub.publish(twist)

            elif self.running_mode == 2:
                min_index_left, min_index_right = np.nanargmin(np.array(left_ranges)),  np.nanargmin(np.array(right_ranges))
                min_dist_left, min_dist_right = left_ranges[min_index_left], right_ranges[min_index_right]
                if min_dist_left <= self.threshold or min_dist_right <= self.threshold: # obsstacle detected
                    self.buzzer_pub.publish(0.1)

    def cleanup(self):
        # global is_shutdown
        # is_shutdown = True
        self.lidar_sub.unregister()
        self.velocity_pub.publish(geo_msg.Twist())
        print('is_shutdown')


if __name__ == "__main__":
    node = LidarController('lidar_obj_avoidance')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
