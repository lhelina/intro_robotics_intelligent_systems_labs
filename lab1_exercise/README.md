# ROS 2 Lab 1 – Robot Sensors, Obstacle Avoidance, and Actions

## 1. Project Overview

This project extends the differential-drive robot with additional sensors, custom ROS 2 interfaces, obstacle avoidance, and action-based movement.

The robot uses:

- A depth camera that produces depth images and point clouds.
- An IMU that provides orientation and motion data.
- An obstacle avoidance node that controls the robot.
- Custom messages, services, and actions.
- An action client that drives the robot in a square.

## 2. Package Structure

The exercise contains four packages:

| Package | Description |
|---|---|
| `diffdrive_bot` | Robot model, sensors, Gazebo simulation, and existing controllers |
| `lab_interfaces` | Existing obstacle information message and other interfaces |
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

## 5. Custom Interfaces

The `my_robot_interfaces` package contains the following interfaces.

**`RobotStatus.msg`** — publishes the robot's current state and obstacle distance:

```text
string state
float32 obstacle_distance
```

**`SetTurnSpeed.srv`** — changes the robot's turning speed:

```text
float32 turn_speed
---
bool success
string message
```

**`Rotate.action`** — requests a rotation and provides feedback about the angle turned:

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

Run the node with:

```bash
ros2 run my_robot_nodes turn_away
```

## 7. Rotation Action Server

The `rotate_server` node:

- Provides the `/rotate` action.
- Uses `/odom` to measure the robot's rotation.
- Supports positive angles for left turns and negative angles for right turns.
- Publishes feedback while rotating.
- Stops the robot when the goal completes or is cancelled.

Run the node with:

```bash
ros2 run my_robot_nodes rotate_server
```

## 8. Square Movement

The `square_driver` node uses:

- `/drive_distance` to move forward.
- `/rotate` to turn left by 90 degrees.

Run the complete simulation with:

```bash
ros2 launch diffdrive_bot square.launch.py
```

To change the side length, run:

```bash
ros2 launch diffdrive_bot square.launch.py side_length:=0.8
```

The default side length is 0.5 meters, and the maximum allowed side length is 1 meter.

## 9. Sensors

### Depth Camera

The depth camera publishes:

- `/depth_camera/image`
- `/depth_camera/points`
- `/depth_camera/camera_info`

### IMU

The IMU publishes:

- `/imu`

## 10. Screenshots

<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/99562683-ba67-4f32-b08b-b9b8369b92ba" />

<img width="601" height="410" alt="image" src="https://github.com/user-attachments/assets/9a2228bd-b113-413f-8e43-5ba44ee0544e" />

<img width="500" height="250" alt="image" src="https://github.com/user-attachments/assets/e9869c19-d3bc-4d80-83a3-c2f1e4427025" />

<img width="1000" height="650" alt="image" src="https://github.com/user-attachments/assets/ff9e15ff-cac1-423d-ae60-320ab77e5134" />

## 11. Notes and Issues

Add any implementation issues, errors encountered, and their solutions after the final build and simulation.

## 12. Repository

GitHub repository:

```text
https://github.com/lhelina/intro_robotics_intelligent_systems_labs
```
