#!/usr/bin/env python2

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import math
import numpy as np

from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    WALL_TOPIC = "/wall"
    SIDE = rospy.get_param("wall_follower/side") # +1 is left, -1 is right
    SIDE_LEFT = 1
    SIDE_RIGHT = -SIDE_LEFT
    SIDE_FRONT = 0
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    DEBUG = False
    MAX_STEERING = 0.34

    KP = 12.0 if VELOCITY < 3.0 else 5.0
    KI = 0.0
    KD = 0.5 if VELOCITY < 3.0 else 0.0

    def __init__(self):
        self.scan_subscriber = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        self.drive_publisher = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.line_publisher = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)

        self.velocity = self.VELOCITY
        self.steering_angle = 0.0

        self.distances = [0.0, 0.0, 0.0]
        self.error = 0.0
        self.last_error = 0.0
        self.d_error = 0.0
        self.last_time = 0.0

        self.left_wall = False
        self.right_wall = False
        self.front_wall = False

        self.scan_partition = 0.33
        self.left_scan_bounds = [0, 0]
        self.right_scan_bounds = [0, 0]
        self.front_scan_bounds = [0, 0]

        self.scan_ranges = None
        self.left_scan_ranges = None
        self.right_scan_ranges = None
        self.front_scan_ranges = None
        self.visualize_side = self.SIDE

        self.scan_angles = None
        self.left_scan_angles = None
        self.right_scan_angles = None
        self.front_scan_angles = None
        self.scan_angle_bounds = [0.0, 0.0]
        self.scan_angle_increment = 0.0
        self.side_angle_partition_size = 0
        self.front_angle_partition_size = 0

    def best_fit_line(self, distances, angles, side, color, visualize):
        # convert polar coordinates from LIDAR scan to cartesian coordinates
        filtered_dists = distances[distances < self.DESIRED_DISTANCE * 3]
        filtered_angles = distances[distances < self.DESIRED_DISTANCE * 3]

        x = filtered_dists * np.cos(filtered_angles)
        y = filtered_dists * np.sin(filtered_angles)

        f = np.polyfit(x, y, 1)

        if visualize:
            step_size = 0.5 * (int(x[0] < x[-1]) * 2 - 1)
            swap_threshold = step_size * 10
            xf = np.arange(x[0], x[-1], step_size)
            yf = f[0] * xf + f[1]
            if len(xf) < swap_threshold:
                step_size = 0.5 * (int(y[0] < y[-1]) * 2 - 1)
                yf = np.arange(y[0], y[-1], step_size)
                xf = (yf - f[1])/f[0]

            VisualizationTools.plot_line(xf, yf, self.line_publisher, color=color, frame="/laser")

        fp = np.array([-1/f[0], 0])

        xw = -(f[0] * f[1])/(f[0]**2 + 1)
        yw = f[0] * xw + f[1]

        d = (xw**2 + yw**2)**0.5

        return d

    def callback(self, data):
        self.scan_ranges = np.array(data.ranges)

        if self.scan_angles is None:
            self.scan_angle_bounds = [data.angle_min, data.angle_max]
            self.scan_angle_increment = data.angle_increment
            self.scan_angles = np.arange(self.scan_angle_bounds[0], self.scan_angle_bounds[1], self.scan_angle_increment)

            self.side_angle_partition_size = int((abs(self.scan_angle_bounds[0]) * 0.05 + abs(self.scan_angle_bounds[0]))/self.scan_angle_increment)
            self.front_angle_partition_size = int((abs(self.scan_angle_bounds[0]) * 0.25)/self.scan_angle_increment)

            self.left_scan_angles = self.scan_angles[-self.side_angle_partition_size:]
            self.right_scan_angles = self.scan_angles[:self.side_angle_partition_size]
            self.front_scan_angles = self.scan_angles[len(self.scan_angles)/2 - self.front_angle_partition_size:(len(self.scan_angles)/2 + self.front_angle_partition_size)]

            if self.DEBUG:
                rospy.loginfo("Range Sizes: " + str(len(self.left_scan_angles)) + " + " + str(len(self.right_scan_angles)) + " + " + str(len(self.front_scan_angles)) + " = " + str(len(self.left_scan_angles) + len(self.right_scan_angles) + len(self.front_scan_angles)))


        self.left_scan_ranges = self.scan_ranges[-self.side_angle_partition_size:]
        self.right_scan_ranges = self.scan_ranges[:self.side_angle_partition_size]
        self.front_scan_ranges = self.scan_ranges[self.side_angle_partition_size:-self.side_angle_partition_size]

        if self.SIDE == self.SIDE_LEFT:
            self.distances[0] = self.best_fit_line(self.left_scan_ranges, self.left_scan_angles, side=self.SIDE_LEFT, color=(1.0, 1.0, 1.0), visualize=False and self.visualize_side == self.SIDE_LEFT)
        else:
            self.distances[2] = self.best_fit_line(self.right_scan_ranges, self.right_scan_angles, side=self.SIDE_RIGHT, color=(0.0, 0.0, 1.0), visualize=False and self.visualize_side == self.SIDE_RIGHT)

        self.steer(self.distances)

    def steer(self, distances):
        if self.SIDE == self.SIDE_LEFT:
            self.error = self.SIDE_LEFT * (distances[0] - self.DESIRED_DISTANCE)
        else:
            self.error = self.SIDE_RIGHT * (distances[2] - self.DESIRED_DISTANCE)

        time_now = rospy.get_time()
        self.d_error = (self.error - self.last_error)/(time_now - self.last_time)
        self.last_error = self.error
        self.last_time = time_now

        steering = self.KP * self.error + self.KD * self.d_error

        if self.DEBUG:
            rospy.loginfo("Distances: " + str(distances) + " Error: " + str(self.error) + " KP: " + str(self.KP) + " KD: " + str(self.KD) + " Steering: " + str(steering))

        ackermann_drive = AckermannDriveStamped()
        ackermann_drive.header.stamp = rospy.Time.now()
        ackermann_drive.header.frame_id = "map"

        ackermann_drive.drive.speed = self.VELOCITY
        ackermann_drive.drive.steering_angle = steering

        self.drive_publisher.publish(ackermann_drive)



if __name__ == "__main__":
    rospy.init_node("wall_follower")
    wall_follower = WallFollower()
    rospy.spin()
