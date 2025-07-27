# Control Lab

This package contains scripts for robot motion control and state estimation. It features a PID-based path follower and a custom odometry estimator using a Runge-Kutta integration method.

## Scripts

- **`pid_follow_lab_student.py`**: This is the main script for a ROS node that enables a robot to follow various predefined paths (e.g., ellipse, spiral, heart-shaped). It uses a PID controller to calculate the necessary linear and angular velocities to minimize the error between the robot's current position and the target path. It subscribes to the custom odometry topic `/range_kutta_odom` for pose information.

- **`kutta.py`**: This script implements a custom odometry estimation node. It subscribes to `/joint_states` (for wheel encoder data) and `/imu` (for angular velocity) and uses a Runge-Kutta integration method to estimate the robot's pose (`x`, `y`, `yaw`). The estimated pose is then published as an `Odometry` message to the `/range_kutta_odom` topic. It also calculates and logs the Mean Squared Error between its estimate and the ground truth `/odom` topic.

## How to Use

1.  Launch your robot simulation in Gazebo.
2.  Run the `kutta.py` script to start the custom odometry estimation. This will publish pose estimates to the `/range_kutta_odom` topic.
3.  Run the `pid_follow_lab_student.py` script to start the PID path following controller. You can specify the path type (e.g., `heart`, `ellipse`) as a ROS parameter. The robot will then start following the chosen trajectory.
