Lower level ROS controllers for following joint commands with Franka Emika robot. This code runs on computer attached to the robot with real-time kernel. 

## System Requirements

[1] Ubuntu 18.04

[2] ROS Melodic

[3] `libfranka` v0.7

## Setup

We assume `libfranka` has been setup using the instructions [here](https://frankaemika.github.io/docs/installation_linux.html)

[1] ```mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src```

[2] ```git clone git@gitlab.com:mohakb/franka_motion_control.git```

[3] ```catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build``` 


(Optional) ``source ~/catkin_ws/src/franka_motion_control/config/franka_setup.sh``. This adds a few home configs as environment variables.

## Examples
 
[1] Moving to Joint Configurations

```roslaunch franka_motion_control move_to_joint_config.launch goal_config:="" robot_ip:="" speed_factor=""```

If `franka_setup.sh` was sourced, we can use
```roslaunch franka_motion_control move_to_joint_config.launch goal_config:="${home_1}" robot_ip:="172.16.0.2" speed_factor:="0.2"```


[2] Monitor Mode

Runs robot in monitor mode where it publishes robot state to a ROS topic.

`roslaunch franka_motion_control monitor_mode.launch`

`rostopic echo /franka_motion_control/joint_states`

[3] Gravity Compensation Mode

Runs robot in gravity compensation mode

`roslaunch franka_motion_control gravity_mode.launch `

[4] Tracking Controller

Runs a torque controller that tracks joint position, velocity and acceleration commands sent as sensor_msgs/JointState message.

`roslaunch franka_motion_control command_mode.launch`

