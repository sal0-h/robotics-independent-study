#! /usr/bin/env python

import rospy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np

class OccupancyGridNode:
    def __init__(self):
        # Action rate
        self.frequency_updates = 100.0
        self.Rate = rospy.Rate(self.frequency_updates)
        
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)
        self.scan_msg = None
        
        # BY MANUAL INSPECTION, THE RANGE OF THE SCANNER IS 3.5 METERS
        # SO WE CAN DO A 7 BY 7 METER THING
        self.scanner_range = 3.5
        # meters per cell
        self.resolution = 0.05  
        self.resolutioninv = 20# I dont wanna deal with float issues
        
        # cells
        # NUM_CELLS = (METERS PER CELL)^-1 TIMES METERS
        self.width = int(20 * self.scanner_range * 2)
        self.height = int(20 * self.scanner_range * 2)
        # robot center at (0,0) at all times
        self.origin = (-self.scanner_range / 2, -self.scanner_range / 2)  
        # (-3.5, -3.5)
        
        self.map = np.zeros((self.height, self.width), dtype=np.int8)
        
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
        a_min = msg.angle_min
        a_max = msg.angle_max
        a_inc = msg.angle_increment
        
        for i, r in enumerate(msg.ranges):
            if np.isinf(r):
                continue
            angle = a_min + a_inc * i
            # Get the distaances from the robot
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            # dividing by resolution gives number of cells
            # Need to shift by half of the width since robot middle is in the middle
            # i.e. 0 would need to be in the middle
            i = int(x / self.resolution) + self.width // 2
            j = int(y / self.resolution) + self.height // 2
            assert(type(i) == type(j) == int)
            if i < 0 or i >= self.width:
                continue
            if j < 0 or j >= self.height:
                continue
            # I AM NOT DOING PROBABILITY
            self.map[j][i] = 100
            
    
    def publish_grid(self):
        self.grid.header.stamp = rospy.Time.now()
        self.grid.info.resolution = self.resolution
        self.grid.info.width = self.width
        self.grid.info.height = self.height
        self.grid.info.origin.position.x = self.origin[0]
        self.grid.info.origin.position.y = self.origin[1]
        
        # This does row by row, if im not mistaken
        self.grid.data = self.map.flatten().tolist()
        
        self.grid_pub.publish(self.grid)
    
if __name__ == "__main__":
    rospy.init_node("occupancy_grid_node")
    node = OccupancyGridNode()
    while not rospy.is_shutdown():
        node.publish_grid()
        node.Rate.sleep()
