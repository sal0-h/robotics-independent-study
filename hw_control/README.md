# Hardware Control - Standalone PID Path Follower

This package provides a standalone and configurable PID-based path following controller. It is designed to be more modular than the other controllers in this repository, with separate classes for the PID logic, the path following application, and path generation.

## Scripts

- **`pid_follow_path_standalone_student.py`**: This is the main script that integrates all components to make the robot follow a generated path. It contains three main classes:
    - **`PID`**: A generic and highly configurable PID controller class. It allows for separate tuning of P, I, and D gains for both linear and angular velocity.
    - **`FollowPathPID`**: The main application class that subscribes to odometry, uses the `PID` class to calculate control velocities, and publishes them to the `/cmd_vel` topic. It is responsible for moving the robot towards a series of waypoints.
    - **`PathGenerator`**: A class that can generate different types of geometric paths, such as an ellipse, a spiral, or a predefined sample path.

- **`show_path.py`**: A utility script to visualize the reference path and the path followed by the robot in RViz.

## How to Use

1.  Launch your robot simulation in Gazebo.
2.  Run the `pid_follow_path_standalone_student.py` script.
3.  You can specify the path type and PID gains as ROS parameters on the command line. For example:
    ```bash
    rosrun hw_control pid_follow_path_standalone_student.py _path:=ellipse _p_gain_distance:=0.6
    ```
4.  The robot will start following the specified path using the configured PID controller.
5.  (Optional) Run `show_path.py` to visualize the robot's performance in RViz.
