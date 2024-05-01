# Reynard the Robot ROS 2 Package

This package contains the ROS 2 packages for Reynard the Robot. Reynard the Robot is an example robot
intended to help users learn Robot Raconteur and other industrial communication technology. The full
Reynard the Robot project is available at: https://github.com/robotraconteur/reynard-the-robot

See [https://reynard-the-robot.readthedocs.io/](https://reynard-the-robot.readthedocs.io/) for more documentation.

Two packages are included in this repository:

- `reynard_the_robot_ros2_msgs`: ROS messages and services for Reynard the Robot
- `reynard_the_robot_ros2`: ROS package for Reynard the Robot

## Installation

To install the Reynard the Robot ROS package, clone the repository into your catkin workspace and build it:

```bash
# Install Reynard the Robot Python package. It is not a ROS package, but is required by the ROS package
python -m pip install reynard-the-robot
cd ~/ros_ws/src
git clone https://github.com/johnwason/reynard_the_robot_ros2.git
cd ~/ros_ws
colcon build
```

## Example

Run the example

```bash
ros2 launch reynard_the_robot_ros2 reynard_the_robot.launch
```

Run the client script:

```bash
python ~/ros_ws/src/reynard_the_robot_ros2/reynard_the_robot_ros2/examples/reynard_the_robot_ros2_client.py
```

## Messages and Services

Messages and services are provided by the `reynard_the_robot_ros2_msgs` package.

The `reynard_the_robot_ros2_msgs` package provides the ROS message and service definitions for Reynard the Robot.

### Messages

#### ReynardState

Contains the current state of Reynard the Robot.

- `std_msgs/Header header`: The standard ROS header
- `float64 time`: The current time in seconds from the Reynard server start
- `float64[] robot_position`: The current position of the robot body in meters `[x, y]`
- `float64[] arm_position`: The current position of the robot arm joints in radians `[q1, q2, q3]`
- `float64[] robot_velocity`: The current velocity of the robot body in meters per second `[vel_x, vel_y]`
- `float64[] arm_velocity`: The current velocity of the robot arm joints in radians per second `[q1, q2, q3]`

### Services

#### Teleport

Instantly move Reynard to a new position.

Request:

- `float64 x`: The x position in meters
- `float64 y`: The y position in meters

Response:

- `bool success`: True if the teleport was successful
- `string status_message`: A message describing the error if the teleport failed

#### Drive

Drive at a specified velocity for a specified time.

Request:

- `float64[] vel`: The velocity in meters per second or radians per second `[vel_x, vel_y]` or `[vel_q1, vel_q2, vel_q3]`
- `float64 timeout`: The time to drive in seconds. If -1, the robot will drive indefinitely.
- `bool wait`: If true, the service will wait for the drive to complete before returning.

Response:

- `bool success`: True if the drive was successful
- `string status_message`: A message describing the error if the drive failed

#### SetPosition

Set the position of Reynard's arm joints.

Request:

- `float64[] target_position`: The position of the arm joints in radians `[q1, q2, q3]`

Response:

- `bool success`: True if the arm position was set successfully
- `string status_message`: A message describing the error if the arm position failed to set

#### GetPosition

Get the current position of Reynard's arm joints.

Request:

None

Response:

- `float64[] position`: The current position of the arm joints in radians `[q1, q2, q3]`
- `bool success`: True if the arm position was retrieved successfully
- `string status_message`: A message describing the error if the arm position failed to retrieve

#### SetColor

Set the color of Reynard.

Request:

- `float64 r`: The red component of the color in the range 0 to 1
- `float64 g`: The green component of the color in the range 0 to 1
- `float64 b`: The blue component of the color in the range 0 to 1

Response:

- `bool success`: True if the color was set successfully
- `string status_message`: A message describing the error if the color failed to set

#### GetColor

Get the current color of Reynard.

Request:

None

Response:

- `float64 r`: The red component of the color in the range 0 to 1
- `float64 g`: The green component of the color in the range 0 to 1
- `float64 b`: The blue component of the color in the range 0 to 1
- `bool success`: True if the color was retrieved successfully
- `string status_message`: A message describing the error if the color failed to retrieve

## Nodes

The `reynard_the_robot_ros2` package provides the `reynard_the_robot` node. It contains the following topics and 
services:

### Publishers

#### /reynard/state

Publishes the current state of Reynard the Robot as a `reynard_the_robot_ros_msgs/ReynardState` message.

#### /reynard/new_message

Publishes a `std_msgs/String` message when Reynard receives a new message from the user.

### Subscribers

#### /reynard/say

Subscribes to a `std_msgs/String` message to make Reynard say the message.

### Services

#### /reynard/teleport

Service to teleport Reynard to a new position. Uses the `reynard_the_robot_ros_msgs/Teleport` service type.

#### /reynard/drive_robot

Service to drive Reynard's body at a specified velocity. Uses the `reynard_the_robot_ros_msgs/Drive` service type.

#### /reynard/drive_arm

Service to drive Reynard's arm joints to a specified position. Uses the `reynard_the_robot_ros_msgs/SetPosition` service type.

#### /reynard/set_arm_position

Service to set the position of Reynard's arm joints. Uses the `reynard_the_robot_ros_msgs/SetPosition` service type.

#### /reynard/set_color

Service to set the color of Reynard. Uses the `reynard_the_robot_ros_msgs/SetColor` service type.

#### /reynard/get_color

Service to get the current color of Reynard. Uses the `reynard_the_robot_ros_msgs/GetColor` service type.
