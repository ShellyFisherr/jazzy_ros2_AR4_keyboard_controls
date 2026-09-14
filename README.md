
# annin_4_keyboard_controll

![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)

This package provides keyboard-based Cartesian teleoperation for the Annin Robotics AR4 arm using `ROS 2` and `MoveIt 2`. It includes nodes for reading keyboard input, translating commands, monitoring robot state, and calculating joint commands with Jacobian-based control. The project is based on the arm without the gripper, since a custom gripper was used.

The package is an extension of the [`ar4_ros_driver`](https://github.com/Annin-Robotics/ar4_ros_driver/tree/main) and is intended for experimentation with AR4 simulation and hardware control.

## Overview

The `annin_ar4_keyboard_controll` package provides keyboard-based Cartesian teleoperation for the Annin-Robotics AR4 robotic arm. the code now is based on the MK5 arm, but should be compattible with other models (although this is untested). It is an extension of the [`ar4_ros_driver`](https://github.com/Annin-Robotics/ar4_ros_driver/tree/main) and is based in `ROS 2` with `MoveIt 2` and `ros2_control`

The main control pipelines are:
```text
Keyboard input
    -> Cartesian position offset
    -> MoveIt 2 planning
    -> Joint trajectory
    -> ros2_control
    -> AR4 robot
```

```text
Keyboard Input
    -> Cartesian twist
    -> MoveIt Servo
    -> Joint commands
    -> ros2_control
    -> AR4 robot
```

```text
Keyboard Input
    -> Cartesian translational velocity
    -> Damped least-squares Jacobian inverse
    -> Joint trajectory commands
    -> ros2_control
    -> Ar4 robot
```

The DLS controller currently controls Cartesian translation only, using the linear part of the robot Jacobian. Although joint velocities are calculated internally, it currently publishes them as part of a `JointTrajectory` message and integrates them to position commands.
Direct joint-velocity output is not yet completed, but will be the main way used later in the teleoperation. Orientation control is still under development.

## Installation (ROS 2 Jazzy - Ubuntu 24.04)

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

The installation process consists of the following steps:

1. Insstalling ROS 2 Jazzy
2. Create and build the `ar4_ros_driver` package
3. Clone and build this package
4. Install dependencies
5. Build the workspace

### 1. Install ROS 2 Jazzy
---
Follow the official tutorial for installing ROS 2 Jazzy for Ubuntu 24.04. The installation guide can be found [Here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

Make sure that:
- You complete the Environment Setup step

### 2. Create and build `ar4_ros_driver` package
---
Follow the official driver installation guide from the Annin Robotics [official site](https://anninrobotics.com), or directly from their [GitHub page](https://github.com/Annin-Robotics/ar4_ros_driver/tree/main).

Make sure that:

- You can launch the robot and complete the calibration
- You can launch the virual Gazebo setup and controll it using `MoveIt 2`

### 3. Clone the `annin_ar4_keyboard_controll` package

To start, go to your `ros2` workspace and to the `ar4_ros_driver` folder using:
```
cd ~/ros2_ws/src/ar4_ros_driver
```
From here, clone the workspace using
```
To be added at later date, sorry
```
The workspace should now look like
```
ros2_ws
|── build
|── install
|── log
|── src
    |── ar4_ros_driver
        |── annin_ar4_keyboard_controll
        |── other packages...
```
### 4. Install dependencies
---
Initialize and update using `rosdep` (only required to do once per system)
```
sudo rosdep init
rosdep update
```
Install all package dependencies for the workspace
```
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```
This will install all needed dependencies in case any were still missing.

From here, everything should be working since `ros2_control` and `Moveit 2` should already have been installed when the `ar4_ros_driver` package was installed.

### 5.Build the workspace
---
From here, the workspace can be build using:
```
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build
```
Overlay the workspace so it can be accessed:
```
source ~/ros2_ws/install/setup.bash
```

It is adviced to add the source commands to the `~/.bashrc` file so that anytime a new terminal is opened, the workspaces are automatically sourced. this can be done using:
```
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```
A reminder is that anytime the workspace is editted, it needs to be rebuild and re-sourced. Setting up an alias that does this automatically is therefore also recommended. The alias `build` will therefore be made which automatically goes to the correct folder, builds the workspace and sources the ~/.bashrc file to include the new or edited files generated.
```
echo "alias build='cd ~/ros2_ws && colcon build && source ~/.bashrc'" >> ~/.bashrc
```
then, source the `.bashrc` file using
```
source ~/.bashrc
```
or simply open up a new terminal. Now, you can simply use `build` to automatically go back to the main workspace folder, preventing filing mistakes, the workspace is automatically build, and the necessary files are automatically sourced.


## Nodes and ROS Interfaces

| Node | Publishes | Subscribes | Services/Clients |
|---|---|---|---|
| `keyboard_input_reader` | `/keyboard_inputs`<br>`std_msgs/msg/String` | None | None |
| `moveit_position_reader` | `/moveit_position_carthesian`<br>`geometry_msgs/msg/PoseStamped` | None | None |
| `movement_translator` | None | `/keyboard_inputs`<br>`std_msgs/msg/String`<br><br>`/moveit_position_carthesian`<br>`geometry_msgs/msg/PoseStamped` | MoveIt 2 planning and execution |
| `continuous_movement` | `/servo_node/delta_twist_cmds`<br>`geometry_msgs/msg/TwistStamped` | `/keyboard_inputs`<br>`std_msgs/msg/String` | Calls `/servo_node/switch_command_type`<br>`moveit_msgs/srv/ServoCommandType` |
| `dls_reduced_controller` | `/joint_trajectory_controller/joint_trajectory`<br>`trajectory_msgs/msg/JointTrajectory` | `/keyboard_inputs`<br>`std_msgs/msg/String`<br><br>`/joint_states`<br>`sensor_msgs/msg/JointState` | None |

The nodes are independent examples and should not all be run simultaneously. Each control method consumes keyboard input and may command the robot through a different interface:

- `movement_translator` uses MoveIt 2 planning for small Cartesian position changes.
- `continuous_movement` sends Cartesian twist commands to MoveIt Servo.
- `dls_reduced_controller` calculates joint commands using the robot Jacobian and publishes a joint trajectory.

## Executables
Executables can be run using the command:
```
ros2 run annin_ar4_keyboard_controll `Executable`
```
where `Executable` is replaced with one from the following list.
| **Executable**           | **Purpose**                                                                                  |
|--------------------------|----------------------------------------------------------------------------------------------|
| `keyboard_input_reader`  | Read keyboard inputs and publishes key messages                                              |
| `movement_translator`    | Translates keyboard input into movement commands                                             |
| `moveit_position_reader` | Reads or reports MoveIt robot position information                                           |
| `continuous_movement`    | Sends continuous movement commands based on position through MoveIt/Servo-related interfaces |
| `dls_reduced_controller` | Computes joint commands from Cartesian velocity using a DLS Jacobian                         |

To actually run the full system, multiple executables, both from the original `ar4_ros_driver` and from the `annin_ar4_keyboard_controll` package must be run simultaniously.

For position based control through MoveIt (`continuous_movement`), the following five commands must be run, each of them in a separate window:
```
ros2 launch annin_ar4_gazebo gazebo.launch.py use_sim_time:=True
```
then in a new window:
```
ros2 launch annin_ar4_moveit_config moveit.launch.py use_sim_time:=true include_gripper:=False moveit_servo:=True
```
then in another window:
```
ros2 run annin_ar4_keyboard_controll continuous_movement
```
then in yet another window:
```
ros2 run annin_ar4_keyboard_controll moveit_position_reader
```
then, in a final window:
```
ros2 run annin_ar4_keyboard_controll keyboard_input_reader
```

If you want to use the DLS controller (`dls_reduced_controller`) which is also the preffered one, you run the following four commands, each in their own window:
```
ros2 launch annin_ar4_gazebo gazebo.launch.py use_sim_time:=True
```
```
ros2 launch annin_ar4_moveit_config moveit.launch.py use_sim_time:=True include_gripper:=False
```
```
ros2 run annin_ar4_keyboard_controll dls_reduced_controller
```
```
ros2 run annin_ar4_keyboard_controll keyboard_input_reader
```
### Controls
---

Regarding the keyboard controlls, they are as follows:
| **Key** | **Cartesian Direction** |
|---------|-------------------------|
| `W`     | Positive X              |
| `S`     | Negative X              |
| `A`     | Positive Y              |
| `D`     | Negative Y              |
| `R`     | Positive Z              |
| `F`     | Negative Z              |

The Coordinate system is a right-handed system w.r.t. the static `base_link` that is connected to the world.

`movement_translator` was the first version and is no longer used. It is only there for someone who would like to use it as a framework for another project.

## Controller Configurations

There are three controllers that will be explained on how they work.

### `movement_translator`
---
The `movement_translator` setup was the first setup created. It has the simplest mechanism and relies heavily on `MoveIt 2` for the motions. the control path is as follows:
```text
keyboard_input_reader
    -> /keyboard_inputs
    -> /moveit_position_carthesian
    -> movement_translator
    -> goal position altering
    -> path planning
    -> MoveIt
    -> ros2_control
    -> AR4 robot
```
The control path shows that the system reads the current position of the end-effector of the robot, updates the requested goal position based on the keyboard inputs given, of which the possible inputs are listed in the [Controls](#controls) section, sends these positions to `Moveit 2` as a goal position to use for path planning, and when a path is found, sends commands to the AR4 robot to update its position.

it is by far the simplest system implemented, but has the main flaw that it is extremely slow and computationally heavy, as it relies on position control instead of velocity control. The system constantly wants to replan its current route, which is not a stable way to control a system that has to be updated at a high rate to remain stable and accurate, therefor, other methods where created.

### `continuous_movement`
---
The `continuous_movement` node provides continuous Cartesian movement through movement servo.

It subscribes to `/keyboard_inputs`, where keyboard commands are received as `std_msgs/msg/String` messages. A timer runs every 40 ms and publishes `geometry_msgs/msg/TwistStamped` message to `servo_node/delta_twist_cmds`.

The keys control the linear Cartesian velocity as described in [Controls](#controls).

The twist command uses the `base_link` frame and does not include angular velocity. MoveIt Servo receives this Cartesian velocity, calculates the corresponding joint motion, and sends commands through the configured `ros2_control` interfaces.

At startup, the node calls `servo_node/switch_command_type` and requists `TWIST` mode. This tells MoveIt Servo to interpret incomming commands as Cartesian twist commands, which are a combination of linear and angular velocity commands.

The node continuously publishes commands at a set rate. When no valid key input is received, it publishes a set zero twist, stopping the movement. The time limit for a keypress to be received is 0.5 seconds.

Finally, in short, the actual control path is
```text
keyboard_input_reader
    -> /keyboard_inputs
    -> continuous_movement
    -> /servo_node/delta_twist_cmds
    -> MoveIt Servo
    -> joint commands
    -> ros2_control
    -> AR4 robot
```

`continuous_movement` however has one major flaw, which is that it comes to a halt when it nears singularities, such as at the zero joint coordinates with which the system is initiated. Therefor, further development has been discontinued.

### `dls_reduced_controller`
---
This node provides continuous Cartesian motion control of the AR4 robotic arm using the Damped Least Squares (DLS) Inverse Kinematics. MoveIt 2 is used to obtain the robot model, joint group, forward kinematics and Jacobian, while the node itself performs the Cartesian-to-joint movement conversion.

The control path is
```text
keyboard_input_reader
    -> /keyboard_inputs
    -> Cartesian velocity commands
    -> Current joint positions (/joint_states)
    -> MoveIt RobotState
    -> 3x6 Linear Jacobian
    -> DLS inverse
    -> Joint velocities (q̇)
    -> Joint position integration
    -> JointTrajectory
    -> ros2_control
    -> AR4 robot
```
The keybindings are listed in the [Controls](#controls) section.

The control loop itself runs at 100 Hz.

The desired Cartesian velocity is defined as
$$
    \dot{x}_d =
\begin{bmatrix}
v_x & v_y & v_z
\end{bmatrix}^T
$$

The linear part of the robot is used:

$$
    \dot{x}=J_v\dot{q}
$$

Where $J_v$ is a $3\times 6$ matrix. To improve robustness near singular configurations, which was the main problem with `continuous_movement`, the DLS inverse is used:

$$
J^{D} = J_v^\top (J_v J_v^\top + \lambda^2I)^{-1}
$$

The joint velocity is then calculated as:
$$
\dot{q} = J^{D} \dot{x}_d
$$

The calculated joint velocity is integrated over the control period:
$$
    q(k+1) = q(k) + \dot{q}(k)\Delta t
$$

The Cartesian velocity resulting from the calculated joint velocities is:
$$
x_{achieved} = J_v q
$$

$x_{achieved}$ can then be compared with the desired velocity to manually check for any mishaps if needed. It also calculated the end-effector velocity from consecutive `joint_states` measurements and kinematics.



## Parameters
The parameters present in the latest file `dls_reduced_controller` are given below:

-`command_speed` : The preferred speed of the end_effector (currently 0.02 m/s)

-`CONTROL_DT` : The time in between every itteration (currently 0.01 s = 100 Hz)

- `damping_` : the damping constant lambda used for DLS (currently 0.005 = 0.5%)

## Safety behavior
The standard safety measures listed for the AR4 robotic arm remain unaltered, such as the maximum joint velocities and radius of operation.

It however is adviced to have nobody in the vicinity of the arm itself to prevent unforseen accidents from happening.

## Licence
MIT License

Copyright (c) 2026 Daan Krijnen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


