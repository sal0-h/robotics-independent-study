#! /usr/bin/env python

# roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import sys, getopt

# DISCLAIMED: Uses Gemini to annotate the variables names
# How the hell am I supposed to know what _d_gain_int means 
class PID():
    """
    PID controller class for controlling linear and angular velocities.
    """
    def __init__(self, _d_gain, _angle_gain, _d_gain_int, _int_win_len, _int_dt, _v_max, _angle_max):
        """
        Initializes the PID controller.

        Args:
            _d_gain (float): Proportional gain for distance error.
            _angle_gain (float): Proportional gain for angle error.
            _d_gain_int (float): Integral gain for distance error.
            _int_win_len (int): Length of the integral window.
            _int_dt (float): Time step for integral calculation.
            _v_max (float): Maximum linear velocity.
            _angle_max (float): Maximum angular velocity.
        """
        self.d_gain = float(_d_gain)
        self.angle_gain = float(_angle_gain)
        self.d_gain_int = float(_d_gain_int)
        self.int_win_len = int(_int_win_len)
        self.int_dt = float(_int_dt)
        self.int_win = [float(0) for _ in range(self.int_win_len)]
        self.int_win_head = int(0)
        self.int_err = float(0.0)
        self.v_max = float(_v_max)
        self.omega_max = float(_angle_max)

    def update_integral_error(self, err):
        """
        Updates the integral error.

        Args:
            err (float): Current error.
        """
        if self.int_win_head >= self.int_win_len:
            self.reset_integral_error() 
        self.int_win[self.int_win_head] = err
        self.int_win_head += 1
        
        # Calculate the sum of the integral window
        integral_sum = sum(self.int_win)

        # Update the integral error
        self.int_err = integral_sum * self.int_dt

    def reset_integral_error(self):
        """
        Resets the integral error and integral window.
        """
        self.int_win = [float(0) for _ in range(self.int_win_len)]
        self.int_win_head = 0
        self.int_err = 0.0
        
    def get_linear_velocity(self, err_d, err_d_int):
        # Combined integral error with proportioal error 
        lin_vel = err_d * self.d_gain + err_d_int * self.d_gain_int
        if lin_vel < 0:
            return max(lin_vel, -self.v_max)
        else:
            return min(lin_vel, self.v_max)

    def get_angular_velocity(self, err_angle):
        # This is assuming err_angle is the differrent 
        # between theta* and theta
        normalized_err_angle = np.arctan2(np.sin(err_angle), np.cos(err_angle))
        ang_vel = self.angle_gain * normalized_err_angle
        if ang_vel < 0:
            return max(ang_vel, -self.omega_max)
        else:
            return min(self.omega_max, ang_vel)
        
class FollowPathPID():
    """
    Node for following a predefined path using PID control.
    """
    def __init__(self):
        """
        Initializes the FollowPathPID node.
        """
        rospy.init_node('FollowPathPID', log_level=rospy.INFO) # DEBUG, INFO, WARN, ERROR, FATAL
        
        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_burger')   # use 'mobile_base' for turtlebot2
        print('Model: ', self.robot_model)
        if 'turtlebot2' in self.robot_model:
            self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        else:
            self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.on_shutdown(self.shutdown)
        
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.angular.z = 0.0
        
        # Maximum velocities, used inside PID
        self.lin_vel_max = 0.2
        self.ang_vel_max = 5
        
        # Not sure where this goes 
        self.pos_tolerance = 0.04
        
        # This is the d* from the slides
        self.d_offset = 0.05
        # Playing around, wanna reduce it
        # self.d_offset = 0.01
        
        self.frequency_updates = 100
        self.Rate = rospy.Rate(self.frequency_updates)
        
        # linear_distance_proportional_gain, 
        # angular_error_proportional_gain, 
        # linear_distance_integral_gain, 
        # integral_window_length, 
        # integral_time_step, 
        # max_linear_velocity, 
        # max_angular_velocity
        self.pid = PID(1, 3, 0.9, 50, float(1. / self.frequency_updates), self.lin_vel_max, self.ang_vel_max)
        
        
        # Do we need this?
        self.reset_odo = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        
        self.origin = True
        self.not_reset = True
        self.cnt = 0
        
        # self.odom_sub = rospy.Subscriber('/range_kutta_odom', Odometry, self.callback_odometry, queue_size=1)
        self.odom_sub = rospy.Subscriber('/range_kutta_odom', Odometry, self.callback_odometry, queue_size=1)
        self.odo_pos_x = -999
        self.odo_pos_y = -999
        self.odo_yaw = -999
        print('Wait for first odometry data...')
        while (self.odo_pos_x == self.odo_pos_y == self.odo_yaw):
            self.Rate.sleep()
        print(' done!')
        
        print('Wait for a clock ...')
        while rospy.get_rostime() == 0:
            self.Rate.sleep()
        print(' done!')
        
        print("\nRobot initial pose: (%5.2f, %5.2f, %5.f)" % (self.odo_pos_x, self.odo_pos_y, np.degrees(self.odo_yaw)))
        
        self.show_path = rospy.get_param('~show_path', True)
        if self.show_path:
            self.followed_path_pub = rospy.Publisher("followed_path", Float32MultiArray, queue_size=10)
            self.reference_path_pub = rospy.Publisher("reference_path", Float32MultiArray, queue_size=10)

    def callback_odometry(self, msg):
        self.odo_pos_x = msg.pose.pose.position.x
        self.odo_pos_y = msg.pose.pose.position.y
        self.odo_yaw = self.quaternion_to_euler(msg)
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.odo_angle_y = 2.0 * (q[3] * q[2] + q[0] * q[1]) # siny
        self.odo_angle_x = 1.0 - 2.0 * (q[1]**2 + q[2]**2) # cosy
        if self.origin:
            self.odo_pos_xstart = self.odo_pos_x
            self.odo_pos_ystart = self.odo_pos_y
            self.origin = False
        if self.cnt % 500 == 0:
            print("\n Position: (%5.2f, %5.2f, %5.2f), Yaw (deg): (%5.2f, %5.2f)" % (self.odo_pos_x, self.odo_pos_y, msg.pose.pose.position.z, self.odo_yaw, np.degrees(self.odo_yaw)))
            print("Linear twist: (%5.2f, %5.2f, %5.2f)" % (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z))
            print("Angular twist: (%5.2f, %5.2f, %5.2f)" % (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z))
        self.cnt = self.cnt + 1

    def quaternion_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def init_ellipse(self, amplitude_x=float(1), amplitude_y=float(1)):
        """
        Initializes the ellipse path parameters.

        Args:
            amplitude_x (float): Amplitude of the ellipse in x-direction.
            amplitude_y (float): Amplitude of the ellipse in y-direction.
        """
        self.amplitude_x = amplitude_x
        self.amplitude_y = amplitude_y
        self.s_range = 2.0 * np.pi
        
    def init_spiral(self, r_growth=float(0.5)):
        """
        Initializes the spiral path parameters.

        Args:
            r_growth (float): Growth rate of the spiral.
        """
        self.r_growth = r_growth
        self.s_range = 100

    def get_next_ellipse(self, s=float(0)):
        """
        Calculates the next point on the ellipse path.

        Args:
            s (float): Parameter along the ellipse.

        Returns:
            tuple: (x, y) coordinates of the next point.
        """
        self.x = self.amplitude_x * np.cos(s)
        self.y = self.amplitude_y * np.sin(s)
        print("[s: %4.2f] (%5.2f, %5.2f)" % (s, self.x, self.y))
        return (self.x, self.y)

    def get_next_spiral(self, s=float(0)):
        """
        Calculates the next point on the spiral path.

        Args:
            s (float): Parameter along the spiral.

        Returns:
            tuple: (x, y) coordinates of the next point.
        """
        r = self.r_growth * s
        self.x = r * np.cos(s)
        self.y = r * np.sin(s)
        print("[s: %4.2f] (%5.2f, %5.2f)" % (s, self.x, self.y))
        return (self.x, self.y)
    
    def init_straight(self, length=5.0, angle=0.0):
        self.straight_length = length
        self.straight_angle = angle
        self.s_range = length  # Use length as the s_range for straight path

    def get_next_straight(self, s=0.0):
        self.x = s * np.cos(self.straight_angle)
        self.y = s * np.sin(self.straight_angle)
        print("[s: %4.2f] (%5.2f, %5.2f)" % (s, self.x, self.y))
        return (self.x, self.y)
    
    def init_heart(self, scale=1.0):
        """
        Initializes the heart path parameters.
        """
        self.heart_scale = scale
        self.s_range = 2.0 * np.pi

    def get_next_heart(self, s=float(0)):
        """
        Calculates the next point on the heart path.
        """
        self.x = self.heart_scale * (16 * np.sin(s)**3)
        self.y = self.heart_scale * (13 * np.cos(s) - 5 * np.cos(2 * s) - 2 * np.cos(3 * s) - np.cos(4 * s))
        print("[s: %4.2f] (%5.2f, %5.2f)" % (s, self.x, self.y))
        return (self.x, self.y)
        
    def publish_path(self):
        """
        Publishes the reference and followed paths.
        """
        self.reference_path_pub.publish(Float32MultiArray(data=[self.x, self.y, 0.0, rospy.get_time()]))
        # Here is was using an undefined variable, I assumed it meant odo_pos_x, and y
        self.followed_path_pub.publish(Float32MultiArray(data=[self.odo_pos_x, self.odo_pos_y, self.odo_yaw, rospy.get_time()]))
    
    # This function had to be implemented
    def move_to_point(self, xs, ys):
        # First, find proportional error for velocity
        # I am assuming I use odometry as where I am.
        err_d = np.sqrt((xs - self.odo_pos_x) ** 2 + (ys - self.odo_pos_y) ** 2) - self.d_offset
        
        # Now integral error (this is weird)
        self.pid.update_integral_error(err_d)
        err_d_int = self.pid.int_err
        # Why am I getting the variable from inside the object
        # AND THEN PASSING IT BACK IN????
        self.vel.linear.x = self.pid.get_linear_velocity(err_d, err_d_int)
        
        # Now steering
        theta_desired = np.arctan2(ys - self.odo_pos_y, xs - self.odo_pos_x)
        err_angle = theta_desired - self.odo_yaw
        self.vel.angular.z = self.pid.get_angular_velocity(err_angle)
        # goal reached logic (not looking at angle)
        return abs(err_d) < self.pos_tolerance
        
    def shutdown(self):
        """
        Stops the robot and shuts down the node.
        """
        print("Shutdown!")
        rospy.loginfo("Stop TurtleBot")
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.vel_pub.publish(self.vel)
        rospy.sleep(2)

if __name__ == '__main__':
    try:
        followPID = FollowPathPID()
        path_type = rospy.get_param('~type', 'heart')
        print('Path type: ', path_type)

        if path_type == 'ellipse':
            ds = 0.1
            amplitude_x = 1
            amplitude_y = 3
            followPID.init_ellipse(amplitude_x, amplitude_y)
            followPID.get_next = followPID.get_next_ellipse
            ds = 0.1
        elif path_type == 'spiral':
            r_growth = 0.1
            followPID.init_spiral(r_growth)
            followPID.get_next = followPID.get_next_spiral
            ds = 0.25
        # ADDED THIS FOR DEBUGGING
        elif path_type == 'straight':
            length = 5.0  
            angle = (np.pi / 3)  
            followPID.init_straight(length, angle)
            followPID.get_next = followPID.get_next_straight
            ds = 1  
        elif path_type == 'heart':
            scale = 0.3
            followPID.init_heart(scale)
            followPID.get_next = followPID.get_next_heart
            ds = 0.1
        # ADDED THIS FOR DEBUGGING
        s = 0
        (xs, ys) = followPID.get_next(s)
        while True:
            if followPID.move_to_point(xs, ys) == True:
                s += ds
                (xs, ys) = followPID.get_next(s)
                if s >= followPID.s_range:
                    s = 0
                print("New s: ", s)
            followPID.vel_pub.publish(followPID.vel)
            followPID.Rate.sleep()
            followPID.publish_path()
    except rospy.ROSInterruptException:
        pass