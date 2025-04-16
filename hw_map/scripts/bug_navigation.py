#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Pose,  Quaternion
from nav_msgs.msg import OccupancyGrid


from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

class Bug:
    def __init__(self):
        pass
        
    def grid_callback(self, msg):
        self.occupancy_grid = msg
        
    def goal_callback(self, msg):
        self.rel_x = msg.pose.position.x
        self.rel_y = msg.pose.position.x