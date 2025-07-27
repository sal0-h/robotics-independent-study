# Hardware Mapping and Navigation

This package contains a ROS implementation of the Bug2 navigation algorithm for robot navigation and obstacle avoidance. It includes the main navigation logic, a wall-following service, and helper nodes for goal definition.

## Scripts and Nodes

- **`bug_navigation.py`**: An implementation of the Bug2 navigation algorithm. The robot moves in a straight line towards the goal until it encounters an obstacle. It then circumnavigates the obstacle until it can resume its straight-line path to the goal. This node uses the `wall_follower_server.py` to handle the circumnavigation.

- **`wall_follower_server.py`**: A ROS service that provides wall-following capabilities. When called, this service makes the robot follow a wall at a specified distance until a certain condition is met (e.g., re-encountering the m-line in the Bug2 algorithm). It uses a PID controller to maintain the desired distance from the wall.

- **`pose_to_target.py`**: A helper node that continuously calculates the robot's pose relative to a fixed goal and publishes it to the `/to_target` topic. This provides the necessary input for the `bug_navigation.py` node.

- **`pid.py`**: A generic PID controller class that is used by the other scripts in this package for motion control.

## Services

- **`WallFollower.srv`**: The service definition for the wall follower. It takes the parameters of the m-line and the last known distance to the goal, and returns whether the m-line has been re-encountered.

## How to Use

1.  Launch your robot simulation in Gazebo.
2.  Run the `pose_to_target.py` node to provide a goal for the robot.
3.  Run the `wall_follower_server.py` node.
4.  Run the `bug_navigation.py` node.
5.  The robot will start navigating towards the goal using the Bug2 algorithm.
