#! /usr/bin/env python
########################## BUG 2 ###############################################
import rospy
from geometry_msgs.msg import Pose,  Quaternion, Twist, Vector3
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
import random
from pid import PID

class VFH:
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
        self.rate = rospy.Rate(self.frequency_updates)


        #### Initialization of PID- / control-specific variables
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
            self.rate.sleep()
        print('to_target data received!')

        self.grid_sub = rospy.Subscriber('/local_map', OccupancyGrid, self.grid_callback)

        # wait for an input from the scan topic
        self.grid = None

        print('Wait for first grid data...')
        while (self.grid == None):
            self.rate.sleep()
        print('grid data received!')

        self.initial_goal_pose_x = self.rel_x
        self.initial_goal_pose_y = self.rel_y


    def grid_callback(self, msg):
        self.grid = msg.data

    def goal_callback(self, msg):
        self.rel_x = msg.position.x
        self.rel_y = msg.position.y

    def combine_errors(self, err_d, err_a):
        # O(1), dont worry
        d = {k: int(v) for k, v in self.pid_components.items()}

        self.vel.linear.x = self.pid.get_linear_velocity(err_d, d['pv'], d['iv'], d['dv'])
        self.vel.angular.z = self.pid.get_angular_velocity(err_a, d['po'], d['io'], d['do'])

    def move_to_goal(self):
        # Return true if goal reached. Else, tweak velocities and call it a day
        pass

    def brake(self):
        self.vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel.angular = Vector3(0.0, 0.0, 0.0)
        self.vel_pub.publish(self.vel)
        
    def shutdown(self):
        # Stop the robot by publishing zero velocities.
        rospy.loginfo("Stopping the robot...")
        self.brake()
        rospy.sleep(1)


################################# Main() ########################

if __name__ == '__main__':

    try:
        # DEBUG, INFO, WARN, ERROR, FATAL
        rospy.init_node('bug_navigation', log_level=rospy.INFO) 

        # note that the command line parameters of the PID controller are defined inside FollowPathPID
        navigation = VFH({'pv':True, 'po':True,
                        'iv':True, 'io':False,
                        'dv': False, 'do': False} )
        while not rospy.is_shutdown():
            if navigation.move_to_goal():
                break
            
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