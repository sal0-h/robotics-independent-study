#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf.transformations
import geometry_msgs.msg
import numpy as np
import sensor_msgs.msg

DELTA_T = 0.05
WHEEL_RADIUS = 0.033

def angular_difference(angle1, angle2):
    """Compute the smallest difference between two angles."""
    diff = angle1 - angle2
    return np.arctan2(np.sin(diff), np.cos(diff))

class RangeKuttaOdomPublisher:
    def __init__(self):
        rospy.init_node('range_kutta_odom_publisher', anonymous=True)
        self.pub = rospy.Publisher('/range_kutta_odom', Odometry, queue_size=10)
        self.rate = rospy.Rate(1 / DELTA_T)  # Publish at 1/DELTA_T Hz

        self.wheel_pos = None
        self.old_wheel_pos = None
        self.angular_velocity = 0.0

        self.joint_sub = rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.callback_joint, queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu', sensor_msgs.msg.Imu, self.callback_imu, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)

        # For our estimated pose.
        self.initial_pose_received = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # For ground truth pose from /odom.
        self.gt_x = None
        self.gt_y = None
        self.gt_yaw = None

        # For accumulating squared errors over time.
        self.error_accumulator = np.zeros(3)
        self.error_count = 0

        # Counter for printing error periodically.
        self.iteration = 0

    def odom_callback(self, msg):
        pose = msg.pose.pose
        orientation = pose.orientation
        euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        # Update ground truth pose.
        self.gt_x = pose.position.x
        self.gt_y = pose.position.y
        self.gt_yaw = euler[2]
        
        # Use the first /odom message to set the initial pose for our estimator.
        if not self.initial_pose_received:
            self.x = self.gt_x
            self.y = self.gt_y
            self.yaw = self.gt_yaw
            self.initial_pose_received = True
            rospy.loginfo("Initial Pose from /odom: x={}, y={}, yaw={}".format(self.x, self.y, self.yaw))

    def callback_joint(self, msg):
        # Initialize wheel positions when receiving the first joint message.
        if self.wheel_pos is None:
            self.wheel_pos = list(msg.position)
            self.old_wheel_pos = list(msg.position)
        else:
            self.wheel_pos = list(msg.position)

    def callback_imu(self, msg):
        self.angular_velocity = msg.angular_velocity.z

    def update_pose(self):
        # Wait until we have both initial wheel data and initial odometry.
        if self.wheel_pos is None or self.old_wheel_pos is None or not self.initial_pose_received:
            return
        
        # Calculate wheel displacements.
        delta_s_l = WHEEL_RADIUS * (self.wheel_pos[0] - self.old_wheel_pos[0])
        delta_s_r = WHEEL_RADIUS * (self.wheel_pos[1] - self.old_wheel_pos[1])
        delta_s = (delta_s_l + delta_s_r) / 2.0

        delta_yaw = self.angular_velocity * DELTA_T

        # Using a midpoint integration method (not full RK4).
        self.x += delta_s * np.cos(self.yaw + delta_yaw / 2)
        self.y += delta_s * np.sin(self.yaw + delta_yaw / 2)
        self.yaw += delta_yaw

        # Update the old wheel positions for the next cycle.
        self.old_wheel_pos = list(self.wheel_pos)

    def publish_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        pose = geometry_msgs.msg.Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)
        odom_msg.pose.pose = pose

        twist = geometry_msgs.msg.Twist()
        twist.angular.z = self.angular_velocity
        odom_msg.twist.twist = twist

        self.pub.publish(odom_msg)

    def compute_squared_errors(self):
        """Compute squared error for each component and return as a numpy array."""
        if self.gt_x is None or self.gt_y is None or self.gt_yaw is None:
            return None
        
        error_x = self.x - self.gt_x
        error_y = self.y - self.gt_y
        error_yaw = angular_difference(self.yaw, self.gt_yaw)
        
        squared_errors = np.array([error_x**2, error_y**2, error_yaw**2])
        return squared_errors

    def run(self):
        while not rospy.is_shutdown():
            self.update_pose()
            self.publish_odom()
            squared_errors = self.compute_squared_errors()
            
            self.iteration += 1
            if squared_errors is not None:
                # Accumulate squared errors.
                self.error_accumulator += squared_errors
                self.error_count += 1

                # Every 20 iterations, compute and log the cumulative mean squared errors.
                if self.iteration % 20 == 0:
                    cumulative_mean = self.error_accumulator / self.error_count
                    overall_mse = cumulative_mean.mean()
                    rospy.loginfo("Cumulative Squared Errors (x, y, yaw): {:.6f}, {:.6f}, {:.6f}".format(
                        cumulative_mean[0], cumulative_mean[1], cumulative_mean[2]))
                    rospy.loginfo("Overall Mean Squared Error: {:.6f}".format(overall_mse))
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = RangeKuttaOdomPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
