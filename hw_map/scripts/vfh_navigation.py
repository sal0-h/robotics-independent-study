#! /usr/bin/env python
########################## BUG 2 ###############################################
import rospy
from geometry_msgs.msg import Pose,  Quaternion, Twist, Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
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
        # rel_x and rel_y are relative pose of the goal position w.r.t. the current robot position.
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
        
        # self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback_odometry)

        # # wait for an input from the odometry topic
        # self.odo_x = -999
        # self.odo_y = -999
        # self.odo_yaw = -999

        # print('Wait for first odometry data...')
        # while (self.odo_x == self.odo_y == self.odo_yaw):
        #     self.rate.sleep()
        # print('done!')

        # define constants for VFH
        self.angular_resolution = 5 # degrees
        self.num_sectors = 360 // self.angular_resolution
        
        self.goal_tolerance = 0.1
        self.wide_threshold = 8
        

    # def callback_odometry(self, msg):
    #     self.odo_x = msg.pose.pose.position.x
    #     self.odo_y = msg.pose.pose.position.y
    #     quaternion = msg.pose.pose.orientation
    #     (_, _, self.odo_yaw) = euler_from_quaternion((quaternion.x, quaternion.y, 
    #                                                               quaternion.z, quaternion.w))

    def grid_callback(self, msg):
        self.grid = msg.data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        robot_i = height // 2
        robot_j = width // 2

        self.grid_2d = np.reshape(self.grid, (height, width))
        # should change
        self.hist = np.zeros(self.num_sectors, dtype=float)
        for i in range(height):
            for j in range(width):
                if self.grid_2d[i][j] >= 50:
                    # how many cells away from robot
                    delta_i = i - robot_i
                    delta_j = j - robot_j
                    delta_x = delta_j * resolution
                    delta_y = delta_i * resolution

                    d = np.hypot(delta_x, delta_y)
                    f = np.arctan2(delta_y, delta_x)
                    
                    b = math.floor((f % (2 * math.pi)) / np.deg2rad(self.angular_resolution))
                    
                    M = (self.grid_2d[i][j] ** 2) / (d ** 2)
                    
                    self.hist[b] += M                   

    def goal_callback(self, msg):
        self.rel_x = msg.position.x
        self.rel_y = msg.position.y

    def combine_errors(self, err_d, err_a):
        # O(1), dont worry
        d = {k: int(v) for k, v in self.pid_components.items()}

        self.vel.linear.x = self.pid.get_linear_velocity(err_d, d['pv'], d['iv'], d['dv'])
        self.vel.angular.z = self.pid.get_angular_velocity(err_a, d['po'], d['io'], d['do'])
    
    def move_to_goal(self):
        # If arrvied return, ow.w tweak velocities and continue
        dist_goal = math.hypot(self.rel_x, self.rel_y)
        if dist_goal < self.goal_tolerance:
            self.brake()
            return True

        # ompute goal sector in robot frame
        theta_goal = math.atan2(self.rel_y, self.rel_x)          
        theta_goal = theta_goal % (2 * math.pi)                  
        sector_width = math.radians(self.angular_resolution)
        kgoal = int(theta_goal // sector_width)

        fs = [k for k, v in enumerate(self.hist) if v < self.obstacle_threshold]
        if not fs:
            self.brake()
            # totally blocked
            return "BLOCKED"  

        #  candidate valleys
        N = self.num_sectors
        valleys = []
        # linear runs
        start = fs[0]
        prev = fs[0]
        for k in fs[1:]:
            if k == prev + 1:
                prev = k
            else:
                valleys.append((start, prev))
                start = prev = k
        valleys.append((start, prev))
        # merge wrap‐around
        if valleys[0][0] == 0 and valleys[-1][1] == N-1:
            kn, kf = valleys[-1][0], valleys[0][1] + N
            valleys[0] = (kn, kf)
            valleys.pop()

        # Case 1: goal sector inside a valley?
        for kn, kf in valleys:
            # account for wrap 
            if kn <= kgoal <= kf or kn <= (kgoal + N) <= kf:
                kbest = kgoal
                break
        else:
            # classify wide vs narrow
            wide = [(kn, kf) for kn, kf in valleys if (kf - kn + 1) > self.wide_threshold]
            narrow = [(kn, kf) for kn, kf in valleys if (kf - kn + 1) <= self.wide_threshold]

            def circ_dist(a, b):
                d = abs(a - b)
                return min(d, N - d)

            if wide:
                # Case 2: pick wide valley whose center is closest to kgoal
                centers = [((kn + kf)//2 % N, (kn, kf)) for kn, kf in wide]
                kbest, _ = min(centers, key=lambda t: circ_dist(t[0], kgoal))
            else:
                # Case 3: pick narrow valley midpoint closest to kgoal
                centers = [((kn + kf)//2 % N, (kn, kf)) for kn, kf in narrow]
                kbest, _ = min(centers, key=lambda t: circ_dist(t[0], kgoal))

        theta_best = (kbest + 0.5) * sector_width
        err_a = (theta_best + math.pi) % (2 * math.pi) - math.pi
        err_d = dist_goal

        self.combine_errors(err_d, err_a)
        return False

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
            reached = navigation.move_to_goal()
            if reached == "BLOCKED":
                print("BLOCKED")
                break
            elif reached:
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