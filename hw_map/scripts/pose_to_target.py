#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Pose,  Quaternion
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

class RelativePose:
    def __init__(self):
        
        # Action rate
        self.frequency_updates = 100.0
        self.Rate = rospy.Rate(self.frequency_updates)
        
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odometry_callback)
        
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
        
        # Goal position
        self.goal_x = rospy.get_param("~goal_x", 20)
        self.goal_y = rospy.get_param("~goal_y", 20)
        
        # Publisher
        self.pub = rospy.Publisher("to_target", Pose, queue_size=5)
        self.pose = Pose()
        
    def odometry_callback(self, msg):
        # the prefix odo_ is added to possibly distinguish pose estimation with odometry vs. other sources
        self.odo_x = msg.pose.pose.position.x
        self.odo_y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        (_r, _p, self.odo_yaw) = euler_from_quaternion((quaternion.x, quaternion.y, 
                                                                  quaternion.z, quaternion.w))

        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    
        if not (self.cnt % 500):
           print("\n Position: ({:5.2f}, {:5.2f}, {:5.2f}), Yaw (deg): ({:5.2f}, {:5.2f})".format(
                self.odo_x,
                self.odo_y,
                msg.pose.pose.position.z,
                self.odo_yaw,
                np.degrees(self.odo_yaw)
            ))           

        self.cnt = self.cnt + 1
    
    def publish_relative_pose(self):
        dx = self.goal_x - self.odo_x
        dy = self.goal_y - self.odo_y
        
        # Transform to robot frame by rorating -yaw
        rel_x =  np.cos(-self.odo_yaw) * dx - np.sin(-self.odo_yaw) * dy
        rel_y =  np.sin(-self.odo_yaw) * dx + np.cos(-self.odo_yaw) * dy
        
        self.pose.position.x = rel_x
        self.pose.position.y = rel_y
        self.pose.orientation = Quaternion(0, 0, 0, 1)


        self.pub.publish(self.pose)

if __name__ == "__main__":
    rospy.init_node("pose_to_target")
    node = RelativePose()
    while not rospy.is_shutdown():
        node.publish_relative_pose()
        node.Rate.sleep()
