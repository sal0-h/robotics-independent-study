#! /usr/bin/env python
########################## BUG 2 ###############################################
import rospy

from geometry_msgs.msg import Pose,  Quaternion, Twist, Vector3
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
from pid import PID

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
        self.position_tolerance = 0.04

        # The offset from the waypoint, to ensure a smooth and continuous motion along the path
        self.distance_offset = 0.1

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

        print('Wait for first data...')
        while (self.rel_x == self.rel_y):
            self.Rate.sleep()
        print('done!')

        self.grid_sub = rospy.Subscriber('local_map', OccupancyGrid, self.grid_callback)

        # wait for an input from the to_target topic
        self.occupancy_grid = None

        print('Wait for first data...')
        while (self.occupancy_grid == None):
            self.Rate.sleep()
        print('done!')
        
        self.initial_goal_pose_x = self.rel_x
        self.initial_goal_pose_y = self.rel_y
        
        # I need to define the m line. 
        if self.initial_goal_pose_x != 0:
            self.m = self.initial_goal_pose_y / self.initial_goal_pose_x
        else:
            self.m = self.initial_goal_pose_y / (self.initial_goal_pose_x + 0.1e-7)
        # Moving towards, as opposed to moving around goal
        self.moving_to_goal = True
        
    def grid_callback(self, msg):
        self.occupancy_grid = msg
        
    def goal_callback(self, msg):
        self.rel_x = msg.pose.position.x
        self.rel_y = msg.pose.position.y
        
    def compute_mline_distance(self, x, y):
        # For a line through (0, 0) with slope m: distance = |m*x - y| / sqrt(m^2 + 1)
        return abs(self.m * x - y) / math.sqrt(self.m**2 + 1)
    
    
    def is_path_blocked(self):
        # Use the occupancy grid to determine if there is an obstacle in front of the robot.
        # Here, we simply check a small region (window) around the robot in the grid.
        # This is a placeholder—you may need a more advanced method.
        threshold = 50  # occupancy threshold (0 free, 100 occupied)
        window_size = 5  # number of cells around robot to check
        # Assuming the robot is at the center of the occupancy grid:
        center_i = len(self.occupancy_grid.data) // 2
        blocked_count = 0
        # Row-major error
        grid_data = np.array(self.occupancy_grid.data).reshape((self.occupancy_grid.info.height,
                                                                self.occupancy_grid.info.width))
        start_row = max(0, center_i - window_size)
        end_row = min(grid_data.shape[0], center_i + window_size)
        start_col = max(0, center_i - window_size)
        end_col = min(grid_data.shape[1], center_i + window_size)
        region = grid_data[start_row:end_row, start_col:end_col]
        # If many cells are occupied, consider the path blocked.
        return np.count_nonzero(region > threshold) > (window_size ** 2) * 0.3
    
    def combine_errors(self, err_d, err_a):
        # O(1), dont worry
        d = {k: int(v) for k, v in self.pid_components.items()}
        
        self.vel.linear.x = self.pid.get_linear_velocity(err_d, d['pv'], d['iv'], d['dv'])
        self.vel.angular.z = self.pid.get_angular_velocity(err_a, d['po'], d['io'], d['do'])
    
    def move_direct(self):
        # Direct motion: aim to reduce the distance and angle error toward the goal.
        # Use PID to compute linear and angular speeds.
        distance_error = math.hypot(self.rel_x, self.rel_y)
        angle_error = math.atan2(self.rel_y, self.rel_x)
        
        # Calculate command velocities using PID gains.
        self.combine_errors(distance_error, angle_error)
        
        # Check if there is an obstacle ahead.
        if self.is_path_blocked():
            rospy.loginfo("Path is blocked. Switching to obstacle avoidance (wall-following).")
            self.moving_to_goal = False
        # Check if the goal is reached.
        if distance_error < self.position_tolerance:
            rospy.loginfo("Goal reached!")
            return True  # Indicate that the goal was reached.
        return False  # Not reached yet.
    
    def get_side_distance(self):
        # Extract grid dimensions and resolution
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        resolution = self.occupancy_grid.info.resolution

        # Convert the 1D data array into a 2D numpy array for easier indexing
        grid_data = np.array(self.occupancy_grid.data).reshape((height, width))

        # Determine the robot's position in the grid (assumed to be at the center)
        robot_row = height // 2
        robot_col = width // 2

        # Initialize variables to scan leftwards
        steps = 0
        col = robot_col - 1  # Start checking the cell immediately to the left

        # Define the occupancy threshold (cells with values >= 100 are considered occupied)
        threshold = 100

        # Scan leftwards until an occupied cell is found or the edge of the grid is reached
        while col >= 0:
            if grid_data[robot_row][col] >= threshold:
                break
            steps += 1
            col -= 1

        # Calculate the lateral distance by multiplying the number of free cells by the resolution
        return steps * resolution

    def move_around(self):
        # Obtain the measured lateral distance
        measured_distance = self.get_side_distance()  #TODO

        desired_distance = 0.1  

        # Step 3: Compute the error
        error_distance = measured_distance - desired_distance

        # Use the PID controller to compute the angular correction.
        # Here, we use PID's angular velocity computation function.
        # Choose the PID gains (p, i, d) appropriately, e.g., p=1, i=0.5, d=0.1.
        ang_correction = self.pid.get_angular_velocity(error_distance, p=1, i=1, d=1)

        # Set velocities: maintain a constant, low forward speed, adjust angular velocity based on PID.
        lin_vel = 0.1  # small forward speed during wall-following
        self.vel.linear = Vector3(lin_vel, 0.0, 0.0)
        self.vel.angular = Vector3(0.0, 0.0, ang_correction)

        # Monitor when the robot re-encounters the m-line and switch back to moving_to_goal.
        mline_tol = 0.1
        current_mline_distance = self.compute_mline_distance(self.rel_x, self.rel_y)
        if current_mline_distance < mline_tol:
            rospy.loginfo("Re-encountered m-line. Resuming goal-directed motion.")
            self.moving_to_goal = True
            self.pid.reset_integral_errors()

    
    def move_to_goal(self):
        if self.moving_to_goal:
            goal_reached = self.move_direct()
        else:
            self.move_around()
            goal_reached = False  # while wall-following, we haven't reached the goal.
        return goal_reached

    def shutdown(self):
        # Stop the robot by publishing zero velocities.
        rospy.loginfo("Stopping the robot...")
        self.vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel.angular = Vector3(0.0, 0.0, 0.0)
        self.vel_pub.publish(self.vel)
        rospy.sleep(3)      
            
            
################################# Main() ########################
                            
if __name__ == '__main__':

    try:
        rospy.init_node('bug_navigation', log_level=rospy.INFO) # DEBUG, INFO, WARN, ERROR, FATAL

        # note that the command line parameters of the PID controller are defined inside FollowPathPID
        navigation = Bug({'pv':True, 'po':True,
                        'iv':True, 'io':False,
                        'dv': False, 'do': False} )
        while True:
            if navigation.move_to_goal():
                break
            # what to do if move_to_goal fails?
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
