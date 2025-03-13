# Franka_WS

## Overview
This repository provides ROS 2 packages for controlling single and dual Franka Emika Panda arms using joint position, velocity, torque and impedance controllers. You can use the launch files for real hardware as well as for a simulated environment with fake hardware.

## Installation
Ensure you have ROS 2 installed and sourced. Libfranka version, Robot version, Robot Description version, 

> [!IMPORTANT]  
> Libfranka version = 0.14.1  
> Check compatibility with robot server version  
> Robot Description version = 0.3.0  

```bash
git clone https://github.com/shr-eyas/franka_ws.git
cd ~/frank_ws
colcon build 
source install/setup.bash
```

## Dual Arm Control
### Torque Control
To run torque control for dual-arm setup (in separate terminals):

```bash
ros2 launch fr3_controllers joint_torque_controller.launch.py robot_ip:=192.168.1.11 namespace:=fr3_left
```
```bash
ros2 launch fr3_controllers joint_torque_controller.launch.py robot_ip:=192.168.1.12 namespace:=fr3_right
```

#### With Fake Hardware
If using fake hardware and RViz visualization (in separate terminals):

```bash
ros2 launch fr3_controllers joint_torque_controller.launch.py robot_ip:='dont_care' use_rviz:=true namespace:=fr3_left use_fake_hardware:=true
```
```bash
ros2 launch fr3_controllers joint_impedance_controller.launch.py robot_ip:='dont_care' use_rviz:=true namespace:=fr3_right use_fake_hardware:=true
```

### Impedance Control
To enable impedance control for both arms (in separate terminals):

```bash
ros2 launch fr3_controllers joint_impedance_controller.launch.py robot_ip:=192.168.1.12 namespace:=fr3_right
```
```bash
ros2 launch fr3_controllers joint_impedance_controller.launch.py robot_ip:=192.168.1.11 namespace:=fr3_left
```

## Single Arm Control
### Move to Start
To initialize a single Franka arm:

```bash
ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=192.168.1.11
```

### Torque Control
To enable torque control on a single arm:

```bash
ros2 launch fr3_controllers joint_torque_controller.launch.py robot_ip:=192.168.1.11
```

### Impedance Control
To enable impedance control on a single arm:

```bash
ros2 launch fr3_controllers joint_impedance_controller.launch.py robot_ip:=192.168.1.11
```

## Notes
- Ensure that the correct IP addresses are set for each robot before launching.
- Use `use_fake_hardware:=true` for testing in a simulated environment.
- The `namespace` argument is used to distinguish between multiple robots.

## License
This repository follows the MIT License. See `LICENSE` for more details.

## Contact
For issues or contributions, feel free to raise an issue on the [GitHub repository](https://github.com/shr-eyas/franka_ws).
