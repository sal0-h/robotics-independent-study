#! /usr/bin/env python

import rospy

from collections import deque
import numpy as np

import cv2
import math
import time

from std_msgs.msg import Float32MultiArray

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
        yt = self.convert_pixel(-self.y, self.yl, self.yu, self.height)

        self.pts.appendleft((xt, yt))

        total = len(self.pts) - 1
        if self.buffer_size == -1:
            points_to_show = total
        else:
            points_to_show = min(total, self.buffer_size)
        half = points_to_show // 2

        for i in range(1, points_to_show):
            if i == 1 and points_to_show > 10:
                if self.topic_name == 'followed_path':
                    (headx, heady) = self.pts[0]
                    arrow_len = 30
                    orientation = (
                        int(headx + arrow_len * np.cos(-self.yaw)),
                        int(heady + arrow_len * np.sin(-self.yaw))
                    )
                    cv2.arrowedLine(frame, self.pts[0], orientation, (255, 0, 0), 1, 8)
                else:
                    cv2.circle(frame, self.pts[0], 10, (255, 0, 0), 4)

            if i < half:
                thickness = 2
                color = (0, 255, 255)
                cv2.line(frame, self.pts[i - 1], self.pts[i], color, thickness)
            else:
                thickness = 1
                color = (0, 0, 255)
                cv2.line(frame, self.pts[i - 1], self.pts[i], color, thickness)

        cv2.putText(frame, '(0,0)', (self.width // 2 - 20, self.height // 2 + 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

        cv2.imshow(self.win_name, frame)

if __name__ == '__main__':
    range_val = 8
    xl_flw = -range_val
    xu_flw = range_val
    yl_flw = -range_val
    yu_flw = range_val

    xl_given = -range_val
    xu_given = range_val
    yl_given = -range_val
    yu_given = range_val

    width = 500
    height = 500
    bg_color = 192

    xoffset_flw = 20
    yoffset_flw = 100
    win_flw = "FollowedPath"

    xoffset_given = 10 * xoffset_flw + width
    yoffset_given = 100
    win_given = "ReferencePath"

    topic_flw = "followed_path"
    topic_given = "reference_path"

    follow_path = ShowPath(topic_flw, width, height, bg_color, xl_flw, xu_flw,
                           yl_flw, yu_flw, xoffset_flw, yoffset_flw, win_flw)

    given_path = ShowPath(topic_given, width, height, bg_color, xl_given, xu_given,
                          yl_given, yu_given, xoffset_given, yoffset_given, win_given)

    while True:
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

    cv2.destroyAllWindows()
