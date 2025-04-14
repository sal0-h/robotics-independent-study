#! /usr/bin/env python

# to be executed with:
# roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 

# Import the Python library for ROS
import rospy
import time

# Import the Twist message 
from geometry_msgs.msg import Twist

# Import the Odometry message 
from nav_msgs.msg import Odometry 

# Import the Joint State message
from sensor_msgs.msg import JointState

# Import the IMU message
from sensor_msgs.msg import Imu

from rospy.timer import Timer

import tf
import matplotlib
matplotlib.use("Agg")  # Use non-interactive backend
import matplotlib.pyplot as plt
import numpy as np

from std_msgs.msg import Float32MultiArray

import sys

# for the ground truth
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
# http://docs.ros.org/api/gazebo_msgs/html/srv/GetModelState.html

DELTA_T = 0.05
WHEEL_RADIUS = 0.033

class MoveOpenLoop():
    def __init__(self, plan = []):
        rospy.init_node('MoveOpenLoop', anonymous=False)

        rospy.on_shutdown(self.shutdown)
        
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    
        # Set a publish velocity rate of in Hz
        rate = rospy.get_param('/rate', 200)
        self.rate = rospy.Rate(rate)

        # this must correspond to what the spawn() service has generated in gazebo
        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_burger')
        print('Robot model:', self.robot_model)

        #print 'Starting ground truth service ...'
        print("Wait for service ....")
        rospy.wait_for_service("gazebo/get_model_state")
        print(" ... Got it!")
        self.get_ground_truth = rospy.ServiceProxy("gazebo/get_model_state",
                                                    GetModelState)

        # get the pose from the odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry,
                                         self.callback_odometry, queue_size=1)

        # get velocities to be used for moving the robot
        self.v = rospy.get_param('~lin_velocity', 0.2)
        self.omega = rospy.get_param('~ang_velocity', 0.18)

        print('Robot velocity: ({:.2f}, {:.2f})'.format(self.v, self.omega))

        # Creates a var of msg type Twist for velocity
        self.velocity = Twist()

        # initialize all twist velocities to zero
        self.velocity.linear.x =  0.0
        self.velocity.linear.y =  0.0
        self.velocity.linear.z =  0.0
        self.velocity.angular.x =  0.0
        self.velocity.angular.y =  0.0
        self.velocity.angular.z =  0.0

        # pose variables
        self.odo_x = -999
        self.odo_y = -999
        self.odo_yaw = -999

        # wait for an input from the odometry topic to be sure that pose is correctly set when start
        print('Wait for first odometry data...')
        while ((self.odo_x == -999) or (self.odo_y == -999) or (self.odo_yaw == -999)):
            self.rate.sleep()
        print('done!')

        print("Initial Pose: ({:.2f}, {:.2f}) {:.2f}".format(self.odo_x, self.odo_y, self.odo_yaw))
        
        # Set initial pose to odo pose
        self.x = self.odo_x
        self.y = self.odo_y
        self.yaw = self.odo_yaw
        
        self.wheel_pos = None
        self.old_wheel_pos = None
        
        self.joint_sub = rospy.Subscriber('/joint_states', JointState,
                                         self.callback_joint, queue_size=1)
        
        # wait for an input from the joint topic to be sure that stuff is correctly set when start
        print('Wait for first wheel pos data...')
        while self.wheel_pos == None:
            self.rate.sleep()
        print('done!')

        print("Initial Wheel  Pos: {}".format(self.wheel_pos))
        
        self.angular_velocity = None
        
        # get the imu
        self.imu_sub = rospy.Subscriber('/imu', Imu,
                                         self.callback_imu, queue_size=1)
        
        # wait for an input from the joint topic to be sure that stuff is correctly set when start
        print('Wait for first angular velocity data...')
        while self.angular_velocity == None:
            self.rate.sleep()
        print('done!')

        print("Initial Angular Vel: {:.2}".format(self.angular_velocity))

        # this is the plan to actuate, as a list of translations and rotations
        self.plan = plan[:]

        self.errors_x = []
        self.errors_y = []
        self.errors_yaw = []
        
        self.last_time = time.time()
        # Set timer to call update_pose function every 0.05 seconds (20 Hz)
        self.update_timer = rospy.Timer(rospy.Duration(DELTA_T), 
                                        self.update_pose)

    def check_target_with_ground_truth(self, x, y):
        ground_truth = self.get_ground_truth(self.robot_model, "world")
        real_x, real_y = ground_truth.pose.position.x, ground_truth.pose.position.y
        dist = self.euclidean_distance((real_x, real_y), (x, y))
        return dist

    def update_squared_errors(self):
        assert len(self.errors_x) == len(self.errors_y) == len(self.errors_yaw)
        self.errors_x.append((self.x - self.odo_x) ** 2)
        self.errors_y.append((self.y - self.odo_y) ** 2)
        self.errors_yaw.append((self.yaw - self.odo_yaw) ** 2)
        assert len(self.errors_x) == len(self.errors_y) == len(self.errors_yaw)
    
    def update_pose(self, event):
        if self.old_wheel_pos == None:
            delta_s = 0
            self.old_wheel_pos = self.wheel_pos
        else:
            old_pos = self.old_wheel_pos
            self.old_wheel_pos = self.wheel_pos
            delta_s_l = WHEEL_RADIUS * (self.wheel_pos[0] - old_pos[0])
            delta_s_r = WHEEL_RADIUS * (self.wheel_pos[1] - old_pos[1])
            delta_s = (delta_s_l + delta_s_r) / 2
        self.last_time = time.time()
        yaw = self.yaw
        delta_yaw = self.angular_velocity * DELTA_T
        self.x += delta_s * np.cos(yaw + delta_yaw / 2)
        self.y += delta_s * np.sin(yaw + delta_yaw / 2)
        self.yaw += delta_yaw
        self.update_squared_errors()

    def callback_joint(self, msg):
        self.wheel_pos = msg.position
        
        
    def callback_imu(self, msg):
        self.angular_velocity = msg.angular_velocity.z 

    def callback_odometry(self, msg):
        
        self.odo_x = msg.pose.pose.position.x
        self.odo_y = msg.pose.pose.position.y
        self.odo_yaw = self.quaternion_to_euler(msg)

        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        
        self.odo_angle_y = 2.0 * (q[3] * q[2] + q[0] * q[1]) # siny
        self.odo_angle_x = 1.0 - 2.0 * (q[1]**2 + q[2]**2) # cosy

        
    def quaternion_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        #print "Roll: %5.2f  Pitch: %5.2f  Yaw: %5.2f" % (roll, pitch, yaw)
        return yaw

        
    # compute the straight line distance between two points in 2D    
    def euclidean_distance(self, u, v):
        return np.sqrt( (u[0] - v[0])*(u[0] - v[0]) + (u[1] - v[1])*(u[1] - v[1]) )

    # compute the difference between two angles according to atan2() quadrant rules
    def angle_distance(self, a1, a2):
        #atan2(y1, x1) +- atan2(y2, x2) = atan2(y1*x2 +- y2*x1,  x1 * x2 -+ y1*y2)
        x1 = np.cos(a1)
        y1 = np.sin(a1)

        x2 = np.cos(a2)
        y2 = np.sin(a2)

        sum_a1_a2 = np.arctan2(y1*x2 - y2*x1,  x1*x2 + y1*y2)
        
        # print 'Angle distance: {:.3f}'.format(sum_a1_a2)
        
        return sum_a1_a2

    def angle_to_turn(self, target_x, target_y):
        delta_x = target_x - self.x
        delta_y = target_y - self.y
        target_yaw = np.arctan2(delta_y, delta_x) 
        delta_yaw = target_yaw - self.yaw
        delta_yaw = (delta_yaw + np.pi) % (2 * np.pi) - np.pi
        return delta_yaw
    
    
    def move_straight_for_distance_using_estimate(self, target_distance):
        self.start_xy = (self.x, self.y)

        self.velocity.linear.x = self.v
        self.velocity.angular.z = 0.0
                
        while self.euclidean_distance(self.start_xy,
                                      (self.x, self.y)) < target_distance:
            self.vel_pub.publish(self.velocity)
            self.rate.sleep()

            
    def rotate_for_angle_using_estimate(self, target_rotation):
        self.start_yaw = self.yaw

        if target_rotation < 0:
            self.velocity.angular.z = -self.omega
        else:
            self.velocity.angular.z = self.omega
            
        while abs(self.angle_distance(self.start_yaw, self.yaw)) < abs(target_rotation):
            self.vel_pub.publish(self.velocity)
            self.rate.sleep()

            
    def execute_plan(self):
        for s, step in enumerate(self.plan):

            # first part of a plan step consists in moving for a given distance
            (x_target, y_target) = (step[0], step[1])
            
            target_rotation = self.angle_to_turn(x_target, y_target)            
            self.rotate_for_angle_using_estimate(target_rotation)
            
            # Set angular velocity to zero
            self.velocity.angular.z = 0.0                    
            self.vel_pub.publish(self.velocity)    
            
            print('\t** Completed rotation motion of Step {}**'.format(s))
            
            # Find distance to move
            target_distance = self.euclidean_distance((self.x, self.y), (x_target, y_target))
            
            self.move_straight_for_distance_using_estimate(target_distance)

            # set the linear velocity to zero
            self.velocity.linear.x = 0.0                    
            self.vel_pub.publish(self.velocity)

            print('\t** Completed linear motion of Step {}**'.format(s))        
                                    
        print('------------- Plan execution completed! -------------')

        errors_x = self.errors_x[:]
        errors_y = self.errors_y[:]
        errors_yaw = self.errors_yaw[:]

        MSE_x = sum(errors_x) / len(errors_x)
        MSE_y = sum(errors_y) / len(errors_y)
        MSE_yaw = sum(errors_yaw) / len(errors_yaw)

        print('\t** MSE x: {}**'.format(MSE_x))        
        print('\t** MSE y: {}**'.format(MSE_y))        
        print('\t** MSE yawww: {}**'.format(MSE_yaw))  
        
        # Time axis
        timesteps = np.arange(len(errors_x), dtype=np.int32)

        # Plot errors over time
        plt.figure(figsize=(10, 5))
        plt.plot(timesteps, errors_x, label="Error X", marker="o")
        plt.plot(timesteps, errors_y, label="Error Y", marker="s")
        plt.plot(timesteps, errors_yaw, label="Error Yaw", marker="^")      
        
        # Labels and title
        plt.xlabel("Time Step")
        plt.ylabel("Error")
        plt.title("Error Evolution Over Time")
        plt.legend()
        plt.grid(True)

        # Save plot as an image
        plt.savefig("errors_plot.png", dpi=300, bbox_inches="tight")  # Save with high quality
        plt.close()  # Close figure to prevent memory issues

     # Ensure a smooth shutdown of the robot       
    def shutdown(self):        
        rospy.loginfo("**** Stopping TurtleBot! ****")
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.vel_pub.publish(self.velocity)
        rospy.sleep(1)

        
if __name__ == '__main__':

    # This is a plan for a motion
    plan = [ (1, 0), (1.6, 0.7), (1.6,2), (1.3,2.7), (0.4, 3.2), (1, 4), (2.5, 4),
            (1, 0), (1.6, 0.7), (1.6,2), (1.3,2.7), (0.4, 3.2), (1, 4), (2.5, 4)] 
 
    motion_controller = MoveOpenLoop(plan)

    motion_controller.execute_plan()