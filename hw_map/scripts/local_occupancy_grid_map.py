#! /usr/bin/env python

import rospy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
import numpy as np

class OccupancyGrid:
    def __init__(self):
        # Action rate
        self.frequency_updates = 100.0
        self.Rate = rospy.Rate(self.frequency_updates)
        
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)
        self.scan_msg = None
        
        print('Wait for first scan data...')
        while self.scan_msg == None:
            self.Rate.sleep()
        print('done!')
        
        # Just to be sure that some entity is providing time and time is passing
        print('Wait for a clock ...')
        while rospy.get_rostime() == 0:
            self.Rate.sleep()
        print('done!')
        
        self.grid = OccupancyGrid()
        self.grid_pub = rospy.Publisher("/local_map", OccupancyGrid, queue_size=10)

    def callback_scan(self, msg):
        self.scan_msg = msg
    
    def publish_grid(self):
        self.grid_pub.publish(self.grid)
        