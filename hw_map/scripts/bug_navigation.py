#! /usr/bin/env python
########################## BUG 2 ###############################################
import rospy
from geometry_msgs.msg import Pose,  Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
import random
from pid import PID
from hw_map.srv import WallFollower, WallFollowerRequest

class Bug:
    def __init__(self, _control_components = {'pv': True, 'po': True,
                                            'iv': True, 'io': True,
                                            'dv': True, 'do': True}):
        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_burger')   # use 'mobile_base' for turtlebot2
        print('Robot model: ', self.robot_model)

        if 'turtlebot2' in self.robot_model:
            self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        else:
            self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # What function to call when ctrl + c is issued
        rospy.on_shutdown(self.shutdown)

        # Twist object to set velocities
        self.vel = Twist()
        self.vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel.angular = Vector3(0.0, 0.0, 0.0)

        # Max velocities for the robot
        self.linear_vel_max = rospy.get_param('~max_lin_vel', 0.25)
        self.angular_vel_max = rospy.get_param('~max_ang_vel', 0.85)

        # Action rate
        self.frequency_updates = 100.0
        self.Rate = rospy.Rate(self.frequency_updates)


        #### Initialization of PID- / control-specific variables

        # Minimal distance to a waypoint to declare it as reached
        # TODO Use this
        self.position_tolerance = 0.04

        # set a dictionary that stores the choice of the pid that must be used for control
        self.pid_components = _control_components

        # Create a PID controller object.
        # All the parameters of the controller are defined as input parameters with default values
        # Use _name_of_param:=value to change its value on the command line. Note the prefixing _
        # All parameters are passed explicitly for the sake of being comprehensive

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

        self.goal_sub = rospy.Subscriber('to_target', Pose, self.goal_callback)

        # wait for an input from the to_target topic
        self.rel_x = -999
        self.rel_y = -999

        print('Wait for first to_target data...')
        while (self.rel_x == -999 and self.rel_y == -999):
            self.Rate.sleep()
        print('to_target data received!')

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # wait for an input from the scan topic
        self.left_dist = None
        self.right_dist = None
        self.forward_dist = None

        print('Wait for first scan data...')
        while (self.left_dist is None or self.right_dist is None or self.forward_dist is None):
            self.Rate.sleep()
        print('scan data received!')

        self.initial_goal_pose_x = self.rel_x
        self.initial_goal_pose_y = self.rel_y

        # I need to define the m line.
        if self.initial_goal_pose_x != 0:
            self.m = self.initial_goal_pose_y / self.initial_goal_pose_x
        else:
            self.m = self.initial_goal_pose_y / (self.initial_goal_pose_x + 0.1e-7)
        # Moving towards, as opposed to moving around goal
        self.moving_to_goal = True

        self.forward_dist_threshold = 0.5

        self.left_or_right = None
        self.last_distance_error = None

        # Create a proxy for the wall_follow service
        rospy.wait_for_service('wall_follow')
        self.wall_follower_client = rospy.ServiceProxy('wall_follow', WallFollower)

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
                # I made it 10 so it would start turning regardless
                return float(10)

        self.forward_dist = get_range_at_angle(0.0)
        self.left_dist = get_range_at_angle(math.pi / 2)
        self.right_dist = get_range_at_angle(-math.pi / 2)

    def goal_callback(self, msg):
        self.rel_x = msg.position.x
        self.rel_y = msg.position.y

    def compute_mline_distance(self, x, y):
        # For a line through (0, 0) with slope m: distance = |m*x - y| / sqrt(m^2 + 1)
        return abs(self.m * x - y) / math.sqrt(self.m**2 + 1)

    def is_path_blocked(self):
        return self.forward_dist < self.forward_dist_threshold

    def combine_errors(self, err_d, err_a):
        # O(1), dont worry
        d = {k: int(v) for k, v in self.pid_components.items()}

        self.vel.linear.x = self.pid.get_linear_velocity(err_d, d['pv'], d['iv'], d['dv'])
        self.vel.angular.z = self.pid.get_angular_velocity(err_a, d['po'], d['io'], d['do'])

    def move_direct(self):

        distance_error = math.hypot(self.rel_x, self.rel_y)
        angle_error = math.atan2(self.rel_y, self.rel_x)

        # Calculate command velocities using PID gains.
        self.combine_errors(distance_error, angle_error)

        # Check if there is an obstacle ahead.
        if self.is_path_blocked():
            rospy.loginfo("Path is blocked. Calling wall follower service.")
            self.moving_to_goal = False
            # -1 is left, 1 is right
            self.left_or_right = random.choice([-1, 1])
            self.last_distance_error = distance_error
            return False # Indicate that we are switching to wall following

        # Check if the goal is reached.
        if distance_error < self.position_tolerance:
            rospy.loginfo("Goal reached!")
            return True
        return False

    def move_around(self):
        rospy.loginfo("Calling wall follower service...")
        try:
            request_object = WallFollowerRequest() 
            request_object.m = self.m
            request_object.last_distance_error = self.last_distance_error
            result = self.wall_follower_client(request_object)
            if result.m_line_reached:
                rospy.loginfo("Wall follower service returned: m-line reached.")
                self.moving_to_goal = True
                self.pid.reset_integral_errors()
            else:
                rospy.loginfo("Wall follower service returned: m-line not reached.")
                # Optionally, you could implement a timeout or other logic here
                # if the wall following takes too long.
        except rospy.ServiceException as e:
            rospy.logerr("Wall follower service call failed: %s"%e)
            self.moving_to_goal = True 
            self.pid.reset_integral_errors()

    def move_to_goal(self):
        if self.moving_to_goal:
            return self.move_direct()
        else:
            return self.move_around()

    def shutdown(self):
        # Stop the robot by publishing zero velocities.
        rospy.loginfo("Stopping the robot...")
        self.vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel.angular = Vector3(0.0, 0.0, 0.0)
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)


################################# Main() ########################

if __name__ == '__main__':

    try:
        rospy.init_node('bug_navigation', log_level=rospy.INFO) # DEBUG, INFO, WARN, ERROR, FATAL

        # note that the command line parameters of the PID controller are defined inside FollowPathPID
        navigation = Bug({'pv':True, 'po':True,
                        'iv':True, 'io':False,
                        'dv': False, 'do': False} )
        while not rospy.is_shutdown():
            if navigation.move_to_goal():
                break
            # Only publish velocity if we are in direct move mode
            if navigation.moving_to_goal:
                navigation.vel_pub.publish(navigation.vel)
            navigation.Rate.sleep()
        navigation.shutdown()


    except Exception as e:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = type(e)
        error_msg = str(e)
        rospy.logerr(error_type)
        rospy.logerr(error_msg)