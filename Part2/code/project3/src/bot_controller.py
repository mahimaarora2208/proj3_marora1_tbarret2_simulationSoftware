#!/usr/bin/env python

import rospy
import sys
import tf
import os
import yaml
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import radians, degrees, atan2, pi, sqrt
from sensor_msgs.msg import LaserScan


class BotController(object):
    """
    A controller class to drive a turtlebot in Gazebo.
    """

    def __init__(self, rate=4):
        rospy.init_node('bot_controller')
        rospy.loginfo('Press Ctrl c to exit')
        self._velocity_publisher = rospy.Publisher(
            'cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        # # rospy.Subscriber("scan", LaserScan, self.sensor_callback)

        self._rate = rospy.Rate(rate)
        self._robot_name = 'waffle'
        self._velocity_msg = Twist()
        self._kp_linear = 0.1
        self._kp_angular = 0.5
        self._velocity_msg.linear.x = 0.5
        self._velocity_msg.angular.z = 0.1
        self._current_x_pos = None
        self._current_y_pos = None
        self._current_orientation = None
        self._initial_orientation = None
        self._yaw = None
        # start position of the robot
        self._initial_x_pos = None
        self._initial_y_pos = None

        # set up a tf listener to retrieve transform between the robot and the world
        self._tf_listener = tf.TransformListener()
        # parent frame for the listener
        self._parent_frame = 'odom'
        # child frame for the listener
        self._child_frame = 'base_footprint'

    @property
    def current_x_pos(self):
        return self._current_x_pos

    @current_x_pos.setter
    def current_x_pos(self, x):
        self._current_x_pos = x

    @property
    def current_y_pos(self):
        return self._current_y_pos

    @current_y_pos.setter
    def current_y_pos(self, y):
        self._current_y_pos = y

    @property
    def current_orientation(self):
        return self._current_orientation

    @current_orientation.setter
    def current_orientation(self, orientation):
        self._current_orientation = orientation
    
    def compute_distance(self, x1, y1, x2, y2):
        return sqrt(((x2-x1)**2) + ((y2-y1)**2))


    def odom_callback(self, msg):
        """
        Callback function for the Topic odom

        Args:
            msg (nav_msgs/Odometry): Odometry message
        """
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self.current_x_pos = msg.pose.pose.position.x
        self.current_y_pos = msg.pose.pose.position.y
        self.current_orientation = euler_from_quaternion(quaternion)

    def get_transform(self):
        """
        Get the current pose of the robot in the world frame.
        """
        try:
            now = rospy.Time.now()
            self._tf_listener.waitForTransform(self._parent_frame,
                                               self._child_frame,
                                               now,
                                               rospy.Duration(5))
            (trans, rot) = self._tf_listener.lookupTransform(
                self._parent_frame, self._child_frame, now)
            self.current_x_pos = trans[0]
            self.current_y_pos = trans[1]
            self.current_orientation = rot
            rospy.loginfo("odom: ({},{}), {}".format(
                self.current_x_pos,
                self.current_y_pos,
                self.current_orientation[2]))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logfatal("TF Exception")

    def go_straight_tf(self, distance_to_drive, forward=True):
        """
        Move a robot in a straight line.
        This version uses a TF listener.

        Args:
            distance_to_drive (float): Distance to drive in meter.
            forward (bool, optional): Direction. Defaults to True.
        """
        self.get_transform()
        self._initial_x_pos = self.current_x_pos
        self._initial_y_pos = self.current_y_pos

        linear_velocity = 0.0
        if distance_to_drive > 0:
            linear_velocity = 0.3
        elif distance_to_drive < 0:
            linear_velocity = -0.3

        # compute the driven distance
        driven_distance = self.compute_distance(
            self._initial_x_pos,
            self._initial_y_pos,
            self.current_x_pos,
            self.current_y_pos)

        # keep moving the robot until the distance is reached
        while not rospy.is_shutdown():
            if driven_distance <= abs(distance_to_drive):
                driven_distance = self.compute_distance(
                    self._initial_x_pos,
                    self._initial_y_pos,
                    self.current_x_pos,
                    self.current_y_pos)
                rospy.loginfo("Distance driven: {}".format(driven_distance))
                self.run(linear_velocity, 0)
            else:
                self.run(0, 0)
                break
            self._rate.sleep()

    def go_straight_time(self, distance_to_drive, forward=True):
        """
        Move the robot in a straight line.
        This version uses the formula:
        distance = velocity * (t0 - tcurrent)

        Args:
            distance_to_drive (float): distance to drive in meter.
            forward (bool, optional): Direction. Defaults to True.
        """
        # original time
        rospy.sleep(2)
        t_0 = rospy.Time.now().to_sec()

        linear_velocity = 0.0
        if forward:
            linear_velocity = 0.3
        else:
            linear_velocity = -0.3

        # keep moving the robot until the distance is reached
        while not rospy.is_shutdown():
            # current time
            t_1 = rospy.Time.now().to_sec()
            driven_distance = (t_1 - t_0) * abs(self._velocity_msg.linear.x)
            rospy.loginfo("Distance driven: {}".format(driven_distance))
            if driven_distance <= distance_to_drive:
                self.run(linear_velocity, 0)
            else:
                self.run(0, 0)
                break
            self._rate.sleep()

    def rotate(self, relative_angle):
        """
        Rotate the robot a relative angle

        Args:
            relative_angle (float): Relative angle
        """
        if relative_angle > 0:
            angular_velocity = 0.1
        elif relative_angle < 0:
            angular_velocity = -0.1
        else:
            angular_velocity = 0.0
        rospy.sleep(2.0)
        t0 = rospy.Time.now().to_sec()
        # keep rotating the robot until the desired relative rotation is reached
        while not rospy.is_shutdown():
            self.run(0, angular_velocity)
            self._rate.sleep()
            t_current = rospy.Time.now().to_sec()

            rospy.loginfo("t0: {t}".format(t=t0))
            rospy.loginfo("t1: {t}".format(t=t_current))
            current_angle = (t_current - t0) * angular_velocity
            rospy.loginfo("current angle: {}".format(current_angle))
            if abs(current_angle) >= radians(abs(relative_angle)):
                rospy.loginfo("Relative angle reached")
                self.run(0, 0)
                break

    def go_to_goal(self, goal_x, goal_y):
        """
        Make the robot reach a 2D goal using a proportional controller

        Args:
            goal_x (float): x position
            goal_y (float): y position
        """
        rospy.loginfo("Go to goal ({}, {})".format(goal_x, goal_y))
        # get position and yaw from transform
        self.get_transform()

        distance_to_goal = self.compute_distance(
            self.current_x_pos, self.current_y_pos, goal_x, goal_y)

        while not rospy.is_shutdown():
            move_cmd = Twist()
            if distance_to_goal > 0.05:
                distance_to_goal = self.compute_distance(self.current_x_pos,
                                                    self.current_y_pos, goal_x, goal_y)
                # compute the heading
                angle_to_goal = atan2(
                    goal_y - self.current_y_pos, goal_x - self.current_x_pos)

                rospy.loginfo("Distance to goal: {}".format(distance_to_goal))
                rospy.loginfo("Angle to goal: {}".format(angle_to_goal))

                # Make the robot rotate the correct direction
                if angle_to_goal < 0:
                    angle_to_goal = 2 * pi + angle_to_goal

                # compute relative orientation between robot and goal
                w = angle_to_goal - self.current_orientation[2]
                if w > pi:
                    w = w - 2 * pi

                # proportional control for angular velocity
                move_cmd.angular.z = self._kp_angular * w

                if move_cmd.angular.z > 0:
                    move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
                else:
                    move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

                # proportional control for linear velocity
                move_cmd.linear.x = min(
                    self._kp_linear * distance_to_goal, 0.6)

                self._velocity_publisher.publish(move_cmd)
                self._rate.sleep()
            else:
                rospy.loginfo("Goal reached")
                break

    def run(self, linear, angular):
        """
        Publish linear and angular velocities to cmd_vel Topic.

        Args:
            linear (float): linear velocity
            angular (float): angular velocity
        """
        velocity = Twist()
        velocity.linear.x = linear
        velocity.angular.z = angular
        self._velocity_publisher.publish(velocity)

    def myhook(self):
        """
        Function to call when shutting down a Node
        """
        rospy.loginfo("shutdown time!")

    def handle_inputs(self):
        """
        Handle arguments passed to the command line
        """

        # get the action to perform
        # drive straight, rotate, or go to goal
        action_name = rospy.get_param("~action")

        if action_name == "drive":
            distance = rospy.get_param("~distance")
            if distance > 0:
                self.go_straight_tf(distance, True)
            elif distance < 0:
                self.go_straight_tf(distance, False)
            else:
                rospy.logerr("Distance not provided")
                rospy.on_shutdown(self.myhook)
                sys.exit(1)
        elif action_name == "rotate":
            angle = rospy.get_param("~angle")
            if angle:
                self.rotate(angle)
            else:
                rospy.logerr("Angle not provided")
                rospy.on_shutdown(self.myhook)
                sys.exit(1)
        elif action_name == "goal":
            x = rospy.get_param("~x")
            y = rospy.get_param("~y")
            if x and y:
                self.go_to_goal(x, y)
            else:
                rospy.logerr("x or y is missing")
                rospy.on_shutdown(self.myhook)
                sys.exit(1)
        elif action_name == "broadcast":
            while not rospy.is_shutdown():
                self.broadcast()
        elif action_name == "yaml":
            self.update_yaml()
        else:
            rospy.logerr("Unknown action")
            rospy.on_shutdown(self.myhook)
            sys.exit(1)

    def broadcast(self):
        br = tf.TransformBroadcaster()
        br.sendTransform((0.1, 0.2, 1),
                         tf.transformations.quaternion_from_euler(0, 0, 3.14),
                         rospy.Time.now(),
                         "marker",
                         "/camera_rgb_optical_frame")

    def update_yaml(self):
        fname = "output/test.yaml"
        stream = open(fname, 'r')
        data = yaml.load(stream)
        data['aruco_marker_0']['xyz'] = [0, 10, 0]
        data['aruco_marker_0']['rpy'] = [1, 1, 1]
        with open(fname, 'w') as yaml_file:
            yaml_file.write(yaml.dump(data, default_flow_style=False))


# if __name__ == '__main__':
#     try:
#         controller = BotController()
#         controller.handle_inputs()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Action terminated.")
