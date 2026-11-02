# ROS 2 Lab 1 – Robot Sensors, Obstacle Avoidance, and Actions

## 1. Project Overview

This project extends a differential-drive robot with additional sensors, custom ROS 2 interfaces, obstacle avoidance, and action-based movement.

The robot includes:

- A depth camera that produces depth images and point clouds.
- An IMU that provides orientation and motion data.
- An obstacle avoidance node that controls the robot's movement.
- Custom ROS 2 messages, services, and actions.
- An action client that moves the robot in a square.

## 2. Package Structure

The exercise contains four ROS 2 packages:

| Package | Description |
|---|---|
| `diffdrive_bot` | Robot model, sensors, Gazebo simulation, controllers, and launch files |
| `lab_interfaces` | Obstacle information message and existing interfaces |
| `my_robot_interfaces` | Custom robot status message, turn-speed service, and rotation action |
| `my_robot_nodes` | Python nodes for obstacle avoidance, rotation, and square movement |

## 3. Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic
- Python 3
- Colcon

Source ROS 2 before building:

```bash
source /opt/ros/humble/setup.bash
```

## 4. Build the Workspace

From the workspace root, run:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For every new terminal, source ROS 2 and the workspace again:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 5. Custom Interfaces

The `my_robot_interfaces` package contains three custom interfaces.

**RobotStatus.msg**

Publishes the robot's current state and obstacle distance.

```text
string state
float32 obstacle_distance
```

**SetTurnSpeed.srv**

Changes the robot's turning speed.

```text
float32 turn_speed
---
bool success
string message
```

**Rotate.action**

Requests a rotation and provides feedback about the angle turned.

```text
float32 angle
---
float32 turned
---
float32 turned
```

## 6. Obstacle Avoidance

The `turn_away` node:

- Subscribes to `/obstacle`.
- Publishes velocity commands to `/cmd_vel`.
- Publishes robot status to `/robot_status`.
- Uses `/turn_away_enable` to enable or disable obstacle avoidance.
- Uses `/set_turn_speed` to change the turning speed.

Run the node:

```bash
ros2 run my_robot_nodes turn_away
```

The node starts disabled. Enable obstacle avoidance using:

```bash
ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: true}"
```

Disable it using:

```bash
ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: false}"
```

Change the turning speed using:

```bash
ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 0.7}"
```

These services can be tested while the simulation and `turn_away` node are running.

## 7. Rotation Action Server

The `rotate_server` node:

- Provides the `/rotate` action.
- Uses `/odom` to measure the robot's rotation.
- Supports positive angles for left turns and negative angles for right turns.
- Publishes feedback while rotating.
- Stops the robot when the goal completes or is cancelled.

Run the server separately when testing rotation:

```bash
ros2 run my_robot_nodes rotate_server
```

The square movement launch file starts the rotation server automatically.

## 8. Square Movement

The `square_driver` node uses:

- `/drive_distance` to move forward.
- `/rotate` to turn left by 90 degrees.

Run the complete simulation:

```bash
ros2 launch diffdrive_bot square.launch.py
```

To change the side length:

```bash
ros2 launch diffdrive_bot square.launch.py side_length:=0.8
```

The default side length is 0.5 meters, and the maximum allowed side length is 1 meter.

The robot drives forward four times and performs a left turn after each side.

## 9. Sensors

### Depth Camera

The depth camera publishes:

| Topic | Message Type |
|---|---|
| `/depth_camera/image` | `sensor_msgs/msg/Image` |
| `/depth_camera/points` | `sensor_msgs/msg/PointCloud2` |
| `/depth_camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

### IMU

The IMU publishes:

| Topic | Message Type |
|---|---|
| `/imu` | `sensor_msgs/msg/Imu` |

### Check Sensor Topics

List the available ROS 2 topics:

```bash
ros2 topic list
```

Check sensor publishing rates:

```bash
ros2 topic hz /depth_camera/image
ros2 topic hz /depth_camera/points
ros2 topic hz /imu
```

### Visualize Sensors in RViz

1. Open RViz.
2. Add an Image display for `/depth_camera/image`.
3. Add a PointCloud2 display for `/depth_camera/points`.
4. Set the Fixed Frame to an appropriate available frame, such as `base_link` or `odom`.

## 10. ROS 2 Node Graph

To visualize the nodes and their communication, run the following command in another terminal while the simulation is running:

```bash
ros2 run rqt_graph rqt_graph
```

Select **Nodes/Topics (all)** to display the available nodes and topic connections.

Note: The `square_driver` node exits after completing the square. Therefore, open `rqt_graph` while the robot is still performing the movement to see this node.

## 11. Screenshots

The following screenshots show the project and its implementation.

<img width="500" height="500" alt="Screenshot 1" src="https://github.com/user-attachments/assets/99562683-ba67-4f32-b08b-b9b8369b92ba" />

<img width="601" height="410" alt="Screenshot 2" src="https://github.com/user-attachments/assets/9a2228bd-b113-413f-8e43-5ba44ee0544e" />

<img width="500" height="250" alt="Screenshot 3" src="https://github.com/user-attachments/assets/e9869c19-d3bc-4d80-83a3-c2f1e4427025" />

<img width="1000" height="650" alt="Screenshot 4" src="https://github.com/user-attachments/assets/ff9e15ff-cac1-423d-ae60-320ab77e5134" />

<img width="1000" height="400" alt="Screenshot 5" src="https://github.com/user-attachments/assets/00e19f59-00d9-482b-aca3-789146952f3e" />

## 12. Testing Results and Issues

### Square Movement Test

The square movement simulation completed successfully.

The robot:

- Completed all four driving goals of 0.5 meters each.
- Completed four left turns of approximately 1.59 radians each.
- Successfully reached the final goal.
- Printed `Square completed successfully!`.
- Exited the `square_driver` process cleanly.

### Issues Encountered

**Drive distance action server startup**

Initially, the square driver could not find the `/drive_distance` action server because the drive distance server did not start correctly. The executable startup issue was corrected, and the server was tested successfully.

**Odometry startup**

The drive distance server sometimes needed to wait for odometry before beginning movement. The server was updated to wait for odometry before executing the movement goal.

**RViz timestamp warning**

An RViz warning was observed about messages from the `lidar_link` frame having timestamps older than the available transform data. This warning did not prevent the square movement test from completing successfully.

## 13. Repository

**GitHub Repository:**

https://github.com/lhelina/intro_robotics_intelligent_systems_labs

**Lab 1 Exercise:**

https://github.com/lhelina/intro_robotics_intelligent_systems_labs/tree/main/lab1_exercise
https://github.com/lhelina/intro_robotics_intelligent_systems_labs
```
