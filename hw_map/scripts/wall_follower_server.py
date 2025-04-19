#! /usr/bin/env python

import rospy
from hw_map.srv import WallFollower, WallFollowerResponse
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from pid import PID
import numpy as np
import tf
import math

class WallFollowerService:
    def __init__(self):
        rospy.init_node('wall_follower_service')
        
        # Action rate
        self.frequency_updates = 100.0
        self.rate = rospy.Rate(self.frequency_updates)
        
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5) # Adjust topic if needed

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Wait for initial scan data
        self.left_dist = None
        self.right_dist = None
        self.forward_dist = None
        print('Wait for first scan data...')
        while self.left_dist is None or self.right_dist is None or self.forward_dist is None:
            rospy.sleep(0.1)
        print('Scan data received.')

        self.goal_sub = rospy.Subscriber('to_target', Pose, self.goal_callback)

        # Wait for initial goal data
        self.rel_x = None
        self.rel_y = None
        print('Wait for first goal data...')
        while self.rel_x is None or self.rel_y is None:
            rospy.sleep(0.1)
        print('Goal data received.')

        self.initial_goal_pose_x = self.rel_x
        self.initial_goal_pose_y = self.rel_y

        # Define the m line based on the initial goal
        if self.initial_goal_pose_x != 0:
            self.m = self.initial_goal_pose_y / self.initial_goal_pose_x
        else:
            self.m = self.initial_goal_pose_y / (self.initial_goal_pose_x + 0.1e-7)

        self.last_distance_error = None
        
        self.yaw = -999

        self.odom_sub = rospy.Subscriber('/odom', Odometry,
                                         self.callback_odometry, queue_size=1)
        # wait for an input from the odometry topic to be sure that pose is correctly set when start
        while (self.yaw == -999):
            self.rate.sleep()

        self.wall_follower_service = rospy.Service('wall_follow', WallFollower, self.handle_wall_follow)
        
        rospy.on_shutdown(self.shutdown)
        
        # Max velocities for the robot
        self.linear_vel_max = rospy.get_param('~max_lin_vel', 0.25)
        self.angular_vel_max = rospy.get_param('~max_ang_vel', 0.85)
        _p_gain_distance = rospy.get_param('~p_gain_distance', 0.5)
        _p_gain_angle = rospy.get_param('~p_gain_angle', 3)
        _i_gain_distance = rospy.get_param('~i_gain_distance', 0.9)
        _i_gain_angle = rospy.get_param('~i_gain_angle', 0.1)
        _i_err_window_len = rospy.get_param('~i_err_window_len', 50)
        _i_err_dt = rospy.get_param('~i_err_dt', 1 / self.frequency_updates)
        _d_gain_distance = rospy.get_param('~d_gain_distance', 0.2)
        _d_gain_angle = rospy.get_param('~d_gain_angle', 0.2)
        _v_max = self.linear_vel_max
        _v_ang_max = self.angular_vel_max

        self.pid = PID(_p_gain_distance, _p_gain_angle,
                       _i_gain_distance, _i_gain_angle,
                       _i_err_window_len, _i_err_dt,
                       _d_gain_distance, _d_gain_angle,
                       _v_max, _v_ang_max)
        
        rospy.loginfo("Wall follower service is ready.")
    
    def callback_odometry(self, msg):
        self.yaw = self.quaternion_to_euler(msg)

    def quaternion_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        return yaw
        

    def scan_callback(self, msg):
        self.scan_msg = msg
        a_min = msg.angle_min
        a_inc = msg.angle_increment
        ranges = msg.ranges

        def get_range_at_angle(angle_rad):
            index = int((angle_rad - a_min) / a_inc)
            if 0 <= index < len(ranges):
                return min(ranges[index], float(10))
            else:
                return float(10)

        self.forward_dist = get_range_at_angle(0.0)
        self.left_dist = get_range_at_angle(math.pi / 2)
        self.right_dist = get_range_at_angle(-math.pi / 2)

    def goal_callback(self, msg):
        self.rel_x = msg.position.x
        self.rel_y = msg.position.y

    def compute_mline_distance(self, current_x, current_y):
        return abs(self.m * current_x - current_y) / math.sqrt(self.m**2 + 1)

    def angle_distance(self, a1, a2):
        #atan2(y1, x1) +- atan2(y2, x2) = atan2(y1*x2 +- y2*x1,  x1 * x2 -+ y1*y2)
        x1 = np.cos(a1)
        y1 = np.sin(a1)

        x2 = np.cos(a2)
        y2 = np.sin(a2)

        sum_a1_a2 = np.arctan2(y1*x2 - y2*x1,  x1*x2 + y1*y2)
        
        # print 'Angle distance: {:.3f}'.format(sum_a1_a2)
        
        return sum_a1_a2
    
    def rotate_for_angle_using_odometry(self, target_rotation):
        self.start_yaw = self.yaw
        velocity = Twist()
        if target_rotation < 0:
            velocity.angular.z = -0.2
        else:
            velocity.angular.z = 0.2
            
        while abs(self.angle_distance(self.start_yaw, self.yaw)) < abs(target_rotation):
            self.vel_pub.publish(velocity)
            self.rate.sleep()
    
    def handle_wall_follow(self, req):
        rospy.loginfo("Wall follow service called.")
        target_wall_distance = 0.5
        self.m = req.m
        self.last_distance_error = req.last_distance_error - 0.1
        m_line_reached = False
        linear_speed = 0.1 # m/s for forward motion

        # Initial movement to get close to the wall
        rospy.loginfo("Moving towards the wall...")
        while True:
            forward_dist = self.forward_dist
            if forward_dist > 0.7: # 
                self.move(linear_speed, 0.0) # Move forward slowly
            else:
                rospy.loginfo("Close to the wall, starting rotation.")
                break
            self.rate.sleep()
        self.stop()

        # Rotate 90 degrees
        rospy.loginfo("Rotating 90 degrees...")
        target_rotation = -math.pi / 2.0 # Rotate right
        self.rotate_for_angle_using_odometry(target_rotation)

        rospy.loginfo("Rotation complete, moving along the wall.")

        
        while True:
            left_dist = self.left_dist
            lateral_error = target_wall_distance - left_dist
            angular_vel = self.pid.get_angular_velocity(-lateral_error, p=1, i=1, d=1) 
            self.move(linear_speed, angular_vel)

            # Check for m-line re-encounter using relative x and y
            current_mline_distance = self.compute_mline_distance(self.rel_x, self.rel_y)
            distance_error = math.hypot(self.rel_x, self.rel_y)
            # If the mline is reencountered and it is closer than the last distance error
            if current_mline_distance < 0.1 and distance_error < self.last_distance_error:
                rospy.loginfo("M-line reached (or close enough).")
                m_line_reached = True
                self.stop()
                break
            self.rate.sleep()
        self.pid.reset_integral_errors()
        self.stop()
        return WallFollowerResponse(m_line_reached=m_line_reached)

    def move(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_pub.publish(twist)

    def stop(self):
        self.move(0.0, 0.0)
        
    def shutdown(self):        
        rospy.loginfo("**** Stopping TurtleBot! ****")
        self.stop()
        rospy.sleep(1)

if __name__ == '__main__':
    wall_follower_service = WallFollowerService()
    rospy.spin()