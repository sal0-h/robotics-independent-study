#! /usr/bin/env python

# roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32MultiArray, Empty
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
from collections import deque
import sys

################################# class PID ########################

class PID:
    '''Implement a generic PID controller for the motion control of a mobile robot.
       Given current error measure, it provide two separate methods for returning 
       the output values of linear and angular velocities.
       The initialization method sets all the parameters of the controller. 
       If some components of the P,I,D controller aren't used, the corresponding gains
       can be left set to their default value, which is 0.    
    '''

    def __init__(self,
                 _p_gain_distance=0.0, _p_gain_angle=0.0,
                 _i_gain_distance=0.0, _i_gain_angle=0.0,
                 _i_err_window_len=30, _i_err_dt=0.01,
                 _d_gain_distance=0.0, _d_gain_angle=0.0,
                 _v_lin_max=0.2, _v_ang_max=0.8):
        '''
           Input: all the gains and parameters that are necessary to set up a PID controller.
           _v_max and _angle_max are bounds to the velocities that can be give as output.
           The input arguments are used to define class variables with the same meaning.

           _p_gain_distance: the P gain for the error in the distance component
           _p_gain_angle: the P gain for the error in the angle component
           _i_gain_distance: the I gain for the error in the distance component
           _i_gain_angle: the I gain for the error in the angle component
           _i_err_window_len: the length, as number of error measures, of the time window used
                              to compute the integral for the I part (both for distance and angle)
           _i_err_dt: this is dt, the numeric differential in the numeric computation of the
                       integral ( i_err = \int_t0^t1 err(t)dt )
           _d_gain_distance: the D gain for the error in the distance component
           _d_gain_angle: the D gain for the error in the angle component
        '''
        
        self.p_gain_distance = _p_gain_distance
        self.p_gain_angle = _p_gain_angle

        self.i_gain_distance = _i_gain_distance
        self.i_gain_angle = _i_gain_angle

        self.i_err_window_len = _i_err_window_len 
        self.i_err_dt = _i_err_dt

        # Create two circular buffers to hold the error data for computing integral errors
        # separately for distance and angle. Values are initialized to zero.
        self.i_err_window_data_distance = deque([0] * self.i_err_window_len,
                                                 maxlen=self.i_err_window_len)

        self.i_err_window_data_angle = deque([0] * self.i_err_window_len,
                                              maxlen=self.i_err_window_len)
                
        self.d_gain_distance = _d_gain_distance
        self.d_gain_angle = _d_gain_angle
                
        self.v_max = _v_lin_max
        self.omega_max = _v_ang_max
        
        self.previous_error_distance, self.previous_error_angle = (0, 0)

        
    def update_integral_error_distance(self, err):
        '''The error err is used to update the integral error on distance'''
        self.i_err_window_data_distance.append(err)
        self.i_err_distance = np.sum(self.i_err_window_data_distance) * self.i_err_dt

    def update_integral_error_angle(self, err):
        '''The error err is used to update the integral error on angle'''
        self.i_err_window_data_angle.append(err)
        self.i_err_angle_integral = np.sum(self.i_err_window_data_angle) * self.i_err_dt

    
    def reset_integral_errors(self):
        '''Reset the integral errors, both for distance and angle.
           This is used when a waypoint is reached and a new one shall start.'''
        self.i_err_window_data_distance = deque([0] * self.i_err_window_len,
                                                 maxlen=self.i_err_window_len)

        self.i_err_window_data_angle = deque([0] * self.i_err_window_len,
                                              maxlen=self.i_err_window_len)
        self.i_err_distance = 0
        self.i_err_angle_integral = 0
        
    def bound_v(self, value):
        sign = -1 if value < 0 else 1
        return min(abs(value), self.v_max) * sign
    
    def bound_o(self, value):
        sign = -1 if value < 0 else 1       
        return min(abs(value), self.omega_max) * sign
    
    def get_linear_velocity_P(self, err_d):
        '''Get error in distance and return P component of linear velocity'''
        ##### your code here
        return self.bound_v(self.p_gain_distance * err_d)

    
    def get_linear_velocity_I(self, err_d):
        '''Get error in distance and return I component of linear velocity'''
        ##### your code here
        self.update_integral_error_distance(err_d)
        err_dist_i = self.i_err_distance
        return self.bound_v(self.i_gain_distance * err_dist_i)


    def get_linear_velocity_D(self, err_d):
        '''Get error in distance and return D component of linear velocity'''
        ##### your code here
        derivative = (err_d - self.previous_error_distance) / self.i_err_dt
        self.previous_error_distance = err_d
        return self.bound_v(self.d_gain_distance * derivative)

    
    def get_angular_velocity_P(self, err_angle):
        '''Get error in angle and return P component of angular velocity'''
        ##### your code here
        return self.bound_o(self.p_gain_angle * err_angle)


    def get_angular_velocity_I(self, err_angle):
        '''Get error in angle and return I component of angular velocity'''
        ##### your code here
        self.update_integral_error_angle(err_angle)
        err_angle_i = self.i_err_angle_integral
        return self.bound_o(self.i_gain_angle * err_angle_i)

    def get_angular_velocity_D(self, err_angle):
        '''Get error in angle and return D component of angular velocity'''
        ##### your code here
        derivative = (err_angle - self.previous_error_angle) / self.i_err_dt
        self.previous_error_angle = err_angle
        return self.bound_o(self.d_gain_angle * derivative)

    # The following three empty methods are included for completeness, to deal with a car-like
    # vehicle with two parallel front steering wheels.
    # You don't have to complete them (for now).
    
    # def get_steering_angle_P(self, err_angle):
    #     '''Get error in angle and return P component of steering angle'''
    #     pass
    #     return 0

    # def get_steering_angle_I(self, err_angle):
    #     '''Get error in angle and return I component of steering angle'''
    #     pass
    #     return 0

    # def get_steering_angle_D(self, err_angle):
    #     '''Get error in angle and return D component of steering angle'''
    #     pass
    #     return 0

    
################################# class FollowPathPID ########################

class FollowPathPID:
    '''Given an input path, this class makes use of the PID class to follow the path using feedback control.
       Path is a sequence of Cartesian coordinates which are given from main() one-at-atime by invoking
       the method move_to_point()
    '''

    def __init__(self, _control_components={'pv': True, 'po': True,
                                            'iv': True, 'io': True,
                                            'dv': True, 'do': True}):
        '''
           Initialize all the elements and variables necessary to implement PID-based path following. 
           First, all robot- and ros-specific stuff are initialized, then PID-related stuff.
           Input: _control_components specifies which parts of a PID controller will be used.
                  It is in the form of a dictionary with keys 'pv', 'po', 'iv', 'io', 'dv', 'do' and
                  values True or False. If 'pv' has value True means that the controller uses a 
                  proportional control on (linear) v. If 'po' is True, proportional control is also
                  used on (angular) velocity omega. A similar convention applies to the cases for
                  integral (i) and derivative (d) controls. By default a pi controller is set with
                  the integral part only for linear velocity.
        '''
        
        #### Initialization of robot- and ros-specific variables
        
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
        self.position_tolerance = 0.04

        # The offset from the waypoint, to ensure a smooth and continuous motion along the path
        self.distance_offset = 0.05

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

        # Reset odometry, if possible
        #self.reset_odo = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback_odometry)

        # wait for an input from the odometry topic
        self.odo_x = -999
        self.odo_y = -999
        self.odo_yaw = -999

        # Just a counter to print out information selectively
        self.cnt = 0

        print('Wait for first odometry data...')
        while (self.odo_x == self.odo_y == self.odo_yaw):
            self.Rate.sleep()
        print('done!')
        
        # Just to be sure that some entity is providing time and time is passing
        print('Wait for a clock ...')
        while rospy.get_rostime() == 0:
            self.Rate.sleep()
        print('done!')
                
        print("\nRobot initial pose: ({:5.2f}, {:5.2f}, {:5.0f})".format(
            self.odo_x,
            self.odo_y,
            np.degrees(self.odo_yaw)
        ))
        # Store initial position
        self.odo_xstart = self.odo_x
        self.odo_ystart = self.odo_y
        
        # check if a path-display node using openCV will be active 
        self.show_path = rospy.get_param('~show_path', True)
        if self.show_path:
            # the node will publish followed and reference trajectories on these topics for display
            self.followed_path_pub = rospy.Publisher("followed_path", Float32MultiArray, queue_size=10)
            self.reference_path_pub = rospy.Publisher("reference_path", Float32MultiArray, queue_size=10)

    def callback_odometry(self, msg):
        '''Asynchronous callback function that updates robot's pose each time a new pose update is
           published in the /odom topic.'''

        # the prefix odo_ is added to possibly distinguish pose estimation with odometry vs. other sources
        self.odo_x = msg.pose.pose.position.x
        self.odo_y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        (_r, _p, self.odo_yaw) = euler_from_quaternion((quaternion.x, quaternion.y, 
                                                                  quaternion.z, quaternion.w))

        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        
        self.odo_angle_y = 2.0 * (q[3] * q[2] + q[0] * q[1]) # siny
        self.odo_angle_x = 1.0 - 2.0 * (q[1]**2 + q[2]**2) # cosy

        if not (self.cnt % 500):
           print("\n Position: ({:5.2f}, {:5.2f}, {:5.2f}), Yaw (deg): ({:5.2f}, {:5.2f})".format(
                self.odo_x,
                self.odo_y,
                msg.pose.pose.position.z,
                self.odo_yaw,
                np.degrees(self.odo_yaw)
            ))           
           #print "Linear twist: (%5.2f, %5.2f, %5.2f)" % (msg.twist.twist.linear.x,
           #                                               msg.twist.twist.linear.y, msg.twist.twist.linear.z)
           #print "Angular twist: (%5.2f, %5.2f, %5.2f)" % (msg.twist.twist.angular.x,
           #                                                msg.twist.twist.angular.y, msg.twist.twist.angular.z)

        self.cnt = self.cnt + 1

    # {'pv': True, 'po': True,
    # 'iv': True, 'io': False,
    # 'dv': False, 'do': False}):
    def combine_errors(self, err_d, err_a):
        # O(1), don't worry
        L = [(k, 1 if v else 0) for (k, v) in self.pid_components.items()]
        d = {k : v for (k, v) in L}
        
        self.vel.linear.x = self.pid.get_linear_velocity_P(err_d) * d['pv'] \
                + self.pid.get_linear_velocity_D(err_d) * d['dv'] \
                + self.pid.get_linear_velocity_I(err_d) * d['iv'] \
        
        self.vel.angular.z = self.pid.get_angular_velocity_P(err_a) * d['po'] \
                + self.pid.get_angular_velocity_D(err_a) * d['do'] \
                + self.pid.get_angular_velocity_I(err_a) * d['io'] \
                    
    def move_to_point(self, x, y):
        '''Move to waypoint of coordinates (x,y).
          (x,y) is the next point in the given reference path to follow.
          Keep looping (use self.Rate.sleep()) until the waypoint is reached 
          (accounting for position tolerance and distance offset).
          Return True when the robot arrives to the waypoint.

          Note that the individual velocity contributions must be combined
          additively to obtain the final velocity controls which get published
          (inside this same function) to the /cmd_vel topic.
        '''
        ##### your code here
        # First, find proportional error for velocity
        # I am assuming I use odometry as where I am.
        err_dist_p = np.sqrt((x - self.odo_x) ** 2 + (y - self.odo_y) ** 2) - self.distance_offset     
        
        # Now angular proportional
        theta_desired = np.arctan2(y - self.odo_y, x - self.odo_x)
        err_angle = theta_desired - self.odo_yaw
        err_angle_normalized = np.arctan2(np.sin(err_angle), np.cos(err_angle))
        
        # set velocities
        self.combine_errors(err_dist_p, err_angle_normalized)
        
        # goal reached logic (not looking at angle)
        if abs(err_dist_p) < self.position_tolerance:
            self.pid.reset_integral_errors()
            return True
    

    def publish_path(self, x, y):
        '''If a path display node is active, this function takes care of publishing the relevant
           path information on the topics.'''
        self.reference_path_pub.publish(Float32MultiArray(data=[x, y, 0.0, rospy.get_time()]))

        self.followed_path_pub.publish(Float32MultiArray(data=[self.odo_x, self.odo_y,
                                                               self.odo_yaw,
                                                               rospy.get_time()]))

    def shutdown(self):
        print("Shutdown!")
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")

        self.vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel.angular = Vector3(0.0, 0.0, 0.0)
        
        self.vel_pub.publish(self.vel)

    # sleep just makes sure TurtleBot receives the stop command prior to shutting down 
        rospy.sleep(1)


################################# class PathGenerator ########################
class PathGenerator:
    '''Provide path generators according to different geometries. 
       The path to generate is selected and initialized in the initializer.
    '''
    
    def __init__(self, _path='ellipse', _path_parameters=[]):
        '''Inputs: 
           _path: is a string defining the path type to generate
           kwargs: the arguments to pass to the initialization function for the path
        '''
        
        path_geometries = { 'ellipse': (self.elliptical_path_get_next, self.init_ellipse),
                            'spiral' : (self.spiraling_path_get_next, self.init_spiral),
                            'sample' : (self.sample_path_get_next, self.init_sample) }
        
        self.get_next_point_in_path = path_geometries[_path][0]
        self.path_init = path_geometries[_path][1]
        print("Path parameters:", path_parameters)
        self.path_init(*_path_parameters)

        
    def init_ellipse(self, xc=0.0, yc=0.0, amplitude_x=1.0, amplitude_y=1.0):
        '''Initialize the parameters for generating an elliptical path,
           Inputs:
           xc: x-coord of center
           yc: y-coord of center
           amplitude_x: length of x semi-axis
           amplitude_y: length of y semi-axis
        '''
        print('Init Ellipse: ', xc, yc, amplitude_x, amplitude_y)
        self.amplitude_x = amplitude_x 
        self.amplitude_y = amplitude_y 
        self.xc = xc 
        self.yc = yc 

        
    def elliptical_path_get_next(self, s=0.0):
        '''Generate Cartesian points according to the parametric equation of an ellipse.
           Return the point in path corresponding to the value of the spatial parameter s.
           E.g., s = 0 means the point (xc + amplitude_x, yc) on the ellipse
        '''
        self.x = self.xc + self.amplitude_x * np.cos(s)
        self.y = self.yc + self.amplitude_y * np.sin(s)
        #print "[s: %4.2f] (%5.2f, %5.2f)" % (s, self.x, self.y)
        return (self.x, self.y)
            
    def init_spiral(self, xc=0.0, yc=0.0, r_growth=0.5):
        '''Initialize the parameters for generating a spiraling path.
           Inputs:
           xc: x-coord of center
           yc: y-coord of center
           r_growth: factor of growth of the spiral radius
        '''
        self.r_growth = r_growth
        self.xc = xc 
        self.yc = yc 
    
    def spiraling_path_get_next(self, s=0.0):
        '''Generate Cartesian points according to the parametric equation of an Archimedean spiral.
           Return the point in path corresponding to the value of the spatial parameter s.
        '''
        r = self.r_growth * s
        self.x = self.xc + r * np.cos(s)
        self.y = self.yc + r * np.sin(s)
        #print "[s: %4.2f] (%5.2f, %5.2f)" % (s, self.x, self.y)
        return (self.x, self.y)
        

    def init_sample(self):
        self.sample_path = [ (1.,1.), (2.2, 2.2), (3.5, 2.5), (3.5, 3.1), (4.,4.) ]
        self.sample_path_idx = 0

    def sample_path_get_next(self, s=None):
        self.x, self.y = self.sample_path[self.sample_path_idx]
        self.sample_path_idx += 1
        if self.sample_path_idx == len(self.sample_path):
            print('END of sample path!')
        return (self.x, self.y)
                            

################################# Main() ########################
                            
if __name__ == '__main__':

    try:
        rospy.init_node('follow_path_with_PID', log_level=rospy.INFO) # DEBUG, INFO, WARN, ERROR, FATAL

        # note that the command line parameters of the PID controller are defined inside FollowPathPID
        path_follow_with_PID = FollowPathPID({'pv':True, 'po':True,
                                              'iv':True, 'io':False,
                                              'dv': False, 'do': False} )

        # read the path type from command line
        path_type = rospy.get_param('~path', 'spiral') 
        print('Path type: ', path_type)

        # some default parameters for the paths, can be changed as wished,
        # or passed as command line parameters
        path_parameters = { 'ellipse': (0.0, 0.0, 3.0, 2.0),
                            'spiral' : (0.0, 0.0, 0.5),
                            'sample' : () }
                            
        path_gen = PathGenerator(path_type, path_parameters[path_type])

        s = 0
        ds = 0.2 # increment in the spatialx parameter s
        (xs, ys) = path_gen.get_next_point_in_path(s)
        print('Start at: ', xs, ys)
        while True:
            if path_follow_with_PID.move_to_point(xs, ys) :
                s += ds
                (xs, ys) = path_gen.get_next_point_in_path(s)
            # what to do if move_to_point fails?
            path_follow_with_PID.vel_pub.publish(path_follow_with_PID.vel)
            path_follow_with_PID.Rate.sleep()
            path_follow_with_PID.publish_path(xs, ys)

        
    except Exception as e:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = type(e)
        error_msg = str(e)
        rospy.logerr(error_type)
        rospy.logerr(error_msg)
