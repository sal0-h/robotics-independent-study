#!/usr/bin/env python

# show-path.py: Class that visualizes in parallel a target and a followed path using opencv
# Data points about the two paths are published on two different topics
#
# Author: Gianni Di Caro, 2017
# Free to use and distribute
#
# Usage: in the path generation and control node declare a publisher for topics /followed_path and /reference_path
# with data type Float32MultiArray. E.g.:
#
#        self.points_given_pub = rospy.Publisher('/reference_path', Float32MultiArray, queue_size=10)
#
#        self.points_followed_pub = rospy.Publisher('/followed_path', Float32MultiArray, queue_size=10) 
#
# The computed or given path points, represented as cartesian coordinates (x(s),y(s)), are published on /reference_path,
# while the actually followed path points (x(t),y(t)) (e.g., obtained by odometry) are published on /followed_path.
# Messages are Float32 4-tuples [x y yaw seq], where x and y are the cartesian coordinates, yaw is the orientation of the robot
# in the plane (this is only used in the /followed_path topic), while seq is a sequence number.
# 
# A number of parameters are defined in the __main__ regarding the  visualization windows (width, height, bg color). In particular,
# the parameters  xl, xu, yl, yu specify the max range of values (l=lower, u=upper) that can be taken
# by the cartesian x and y coordinates. A tight definition of these parameters given better / larger path visualization.
# No command line inputs are expected.
#

import rospy
from collections import deque
import numpy as np
import cv2
import math
import time
from std_msgs.msg import Float32MultiArray

# Coordinates in the image: (0,0) of robot's worlds is in the middle
#
# (0,0) --------- (xmax,0)
#   |                |
#   |                |
# (0, ymax) ----- (xmax,ymax)

class ShowPath():

    def __init__(self, topic_name, width, height, bg_color, xl, xu, yl, yu, xoffset, yoffset, win_name):
        
        rospy.init_node('show_path', anonymous=True)

        self.points_sub = rospy.Subscriber(topic_name, Float32MultiArray, self.get_points)

        self.topic_name = topic_name
            
        self.x = 0.0
        self.y = 0.0
        self.seq = -2
        self.last_seq = -1
        
        self.buffer_size = -1
        
        self.pts = deque()

        self.width = width
        self.height = height
        self.bg_color = bg_color

        self.xl = xl
        self.xu = xu
        self.yl = yl
        self.yu = yu
        
        self.win_name = win_name
        cv2.namedWindow(win_name)
        cv2.moveWindow(win_name, xoffset, yoffset)
    
        # just be sure that some entity is providing time and time is passing
        while rospy.get_rostime() == 0:
            rospy.sleep(0.25)

    def convert_pixel(self, pt, pt_low, pt_up, int_scale):
        return int(int_scale * (pt - pt_low) / (pt_up - pt_low))
                      
    def get_points(self, msg):
        self.x = float(msg.data[0])
        self.y = float(msg.data[1])
        self.yaw = float(msg.data[2])
        self.seq = float(msg.data[3])
            
    def draw_grid(self, frame):
        meter_x = int(self.width / (self.xu - self.xl))
        meter_y = int(self.height / (self.yu - self.yl))

        for x in range(self.width // 2, self.width, meter_x):
            cv2.line(frame, (x, 0), (x, self.height), (169, 169, 169), 1)

        for x in range(self.width // 2, 0, -meter_x):
            cv2.line(frame, (x, 0), (x, self.height), (169, 169, 169), 1)
            
        for y in range(self.height // 2, self.height, meter_y):
            cv2.line(frame, (0, y), (self.width, y), (169, 169, 169), 1)

        for y in range(self.height // 2, 0, -meter_y):
            cv2.line(frame, (0, y), (self.width, y), (169, 169, 169), 1)

    def show_in_window(self):
        frame = np.full((self.width, self.height, 3), self.bg_color, dtype=np.uint8)

        self.draw_grid(frame)
        
        xt = self.convert_pixel(self.x, self.xl, self.xu, self.width)
        yt = self.convert_pixel(-self.y, self.yl, self.yu, self.height)  # yscale is upside-down in the image, needs -

        #print "[%s] Got x, y, z: (%5.2f - %d, %5.2f - %d  [%6.3f])" % (self.win_name, self.x, xt, self.y, yt, self.yaw)

        self.pts.appendleft((xt, yt))

        # loop over the set of tracked points
        total = len(self.pts) - 1
        if self.buffer_size == -1:
            points_to_show = total
        else:
            points_to_show = min(total, self.buffer_size)
        half = points_to_show // 2

        for i in range(1, points_to_show):
            #print "Points: ", pts[0], pts[1], pts[total-1], (xt,yt)
            #time.sleep(1)

            if i == 1 and points_to_show > 10:
                if self.topic_name == 'followed_path':
                    #print "YAW: %6.3f [%6.3f]" % (self.yaw, np.degrees(self.yaw))
                    (headx, heady) = self.pts[0]
                    arrow_len = 30
                    # y-coordinates are upsidedown, therefore the yaw must be inverted ( - sign)
                    orientation = (int(headx + arrow_len * np.cos(-self.yaw)), int(heady + arrow_len * np.sin(-self.yaw)))
                    cv2.arrowedLine(frame, self.pts[0], orientation, (255, 0, 0), 1, 8)
                else:
                    cv2.circle(frame, self.pts[0], 10, (255, 0, 0), 4)
                #cv2.FillConvexPoly(frame, , color, lineType=8, shift=0)
            
            # the color (also thickness?) of the line and
            # draw the connecting lines
            if i < half:  # new points
                thickness = 2
                color = (0, 255, 255)
                cv2.line(frame, self.pts[i - 1], self.pts[i], color, thickness)
            else: 
                thickness = 1
                color = (0, 0, 255)
                cv2.line(frame, self.pts[i - 1], self.pts[i], color, thickness)

        # show origin of robot world
        cv2.putText(frame, '(0,0)', (self.width // 2 - 20, self.height // 2 + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

        # show the frame to our screen
        cv2.imshow(self.win_name, frame)

        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    cv2.destroyAllWindows()

if __name__ == '__main__':

    # some command line inputs would be useful here ...
    # range values of points to plot - given: from parametric eqs, flw: followed in real 
    range_val = 8
    xl_flw = -range_val
    xu_flw = range_val
    yl_flw = -range_val
    yu_flw = range_val
    
    xl_given = -range_val
    xu_given = range_val
    yl_given = -range_val
    yu_given = range_val

    # size of the display windows
    width = 500
    height = 500
    bg_color = 192

    # where the place the windows on the desktop
    xoffset_flw = 20
    yoffset_flw = 100
    win_flw = "FollowedPath"

    xoffset_given = 10 * xoffset_flw + width
    yoffset_given = 100
    win_given = "ReferencePath"

    # the names of the topics where path data are pulished

    topic_flw = "followed_path"
    topic_given = "reference_path"
    
    follow_path = ShowPath(topic_flw, width, height, bg_color, xl_flw, xu_flw, yl_flw, yu_flw, xoffset_flw, yoffset_flw, win_flw)
    
    given_path = ShowPath(topic_given, width, height, bg_color, xl_given, xu_given, yl_given, yu_given,
                          xoffset_given, yoffset_given, win_given)

    while True:
        # data points are read from the topics with the callback functions
        
        if follow_path.seq > follow_path.last_seq:
            follow_path.show_in_window()
            follow_path.last_seq = follow_path.seq
            
        if given_path.seq > given_path.last_seq:
            given_path.show_in_window()
            given_path.last_seq = given_path.seq
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

        rospy.sleep(0.05)
        
    # close any open windows
    cv2.destroyAllWindows()