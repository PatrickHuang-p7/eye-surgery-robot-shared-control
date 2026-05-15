# Robotic Arm Shared Control System

This repository contains a ROS 2 Humble project for shared control of a small robotic arm. A human operator moves a joystick, the robotic gripper holds the joystick, and the system guides the motion inside a virtual safety boundary. When the virtual laser point moves outside the boundary, the controller switches to a machine-dominant return behavior.

中文说明见文末的 [中文补充](#中文补充)。

## Repository Layout

```text
.
├── arm_driver/
│   ├── arm_driver/          # Arm driver, shared control, virtual fixture, and tools
│   ├── config/              # Parameter files for arm_control and pygame_haptic_sim
│   ├── launch/              # ROS 2 launch files
│   └── urdf/                # Arm model
├── sensor_interface/
│   ├── sensor_interface/    # ADS1115, joystick, and FSR acquisition nodes
│   └── config/              # ADC and sensor parameter files
└── README.md
```

Main ROS 2 packages:

- `arm_driver`: serial servo driver, high-level arm control state machine, shared control, and Pygame virtual fixture.
- `sensor_interface`: ADS1115 acquisition, joystick position publishing, and FSR contact/overpressure status publishing.

## System Overview

```text
Joystick -> ADS1115(A2/A3) -> hardware_adc_node -> /joy_position
                                                        |
                                                        v
                                 pygame_haptic_sim_node -> /virtual_fixture_force
                                                        -> /laser_position
                                                        -> /shared_control_feedback

FSR -> ADS1115(A0/A1) -> hardware_adc_node -> /gripper_contact
                                            -> /gripper_overpressure
                                            -> /gripper_sensor_fault

/joy_position + /virtual_fixture_force + FSR states -> arm_control_node -> arm_driver_node
```

The shared control law is divided into two regions:

- Inside the boundary: the controller computes an authority factor `alpha` from the virtual fixture force and blends human input with the safety correction.
- Outside the boundary: `alpha = 1`, so the machine fully takes over and pushes the joystick back toward the safe region.

Default shared-control plane:

- Center: `(x, y, z) = (0.22, 0.00, 0.08) m`
- Workspace: `x = 0.22 ± 0.016 m`, `y = 0.00 ± 0.016 m`
- Fixed height: `z = 0.08 m`

## Requirements

Recommended environment:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- ADS1115 I2C ADC
- UART serial servo robotic arm
- Pygame, NumPy, pandas, matplotlib, seaborn, smbus2

Example dependency installation:

```bash
sudo apt update
sudo apt install -y \
  python3-numpy python3-pygame python3-smbus2 python3-serial \
  python3-matplotlib python3-pandas python3-seaborn \
  ros-humble-rosbag2-py ros-humble-rosidl-runtime-py
```

## Build

Clone this repository into a ROS 2 workspace:

```bash
mkdir -p ~/arm_project_ws/src
git clone <repo-url> ~/arm_project_ws/src/arm_project
cd ~/arm_project_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select arm_driver sensor_interface
source install/setup.bash
```

For SSH or headless sessions:

```bash
export SDL_VIDEODRIVER=dummy
```

## Quick Start

Launch the full system:

```bash
ros2 launch arm_driver system_launch.py
```

Default parameter files:

- `arm_driver/config/arm_control_shared_balanced.yaml`
- `arm_driver/config/pygame_haptic_sim_shared_debug.yaml`
- `sensor_interface/config/hardware_adc_shared_debug.yaml`

Common launch overrides:

```bash
ros2 launch arm_driver system_launch.py \
  arm_port:=/dev/serial0 \
  pygame_sdl_video_driver:=dummy
```

Use a real Pygame window instead of the dummy video driver:

```bash
ros2 launch arm_driver system_launch.py pygame_sdl_video_driver:=x11
```

## Main Commands

Run nodes individually:

```bash
ros2 run arm_driver arm_driver --ros-args -p serial_port:=/dev/serial0
ros2 run sensor_interface hardware_adc
ros2 run arm_driver pygame_haptic_sim
ros2 run arm_driver arm_control
```

Useful topics:

```bash
ros2 topic echo /joy_position
ros2 topic echo /laser_position
ros2 topic echo /virtual_fixture_force
ros2 topic echo /shared_control_feedback
ros2 topic echo /gripper_contact
ros2 topic echo /gripper_overpressure
ros2 topic echo /gripper_sensor_fault
```

Trigger a safe return-home sequence:

```bash
ros2 topic pub --once /arm_control_return_home std_msgs/msg/Bool "{data: true}"
```

## Configurations

Shared-control parameter sets:

- `arm_driver/config/arm_control_shared_balanced.yaml`: default balanced setup.
- `arm_driver/config/arm_control_shared_baseline.yaml`: weak shared control / near-manual baseline.
- `arm_driver/config/arm_control_shared_conservative.yaml`: stronger return behavior.
- `arm_driver/config/arm_control_shared_responsive.yaml`: more responsive behavior.

Sensor parameters:

- `sensor_interface/config/hardware_adc_shared_debug.yaml`

Pygame virtual fixture parameters:

- `arm_driver/config/pygame_haptic_sim_shared_debug.yaml`

## Useful Tools

Preview the virtual boundary logic on macOS or without ROS hardware:

```bash
python3 arm_driver/arm_driver/pygame_keyboard_preview.py
```

Run the offline shared-control parameter sweep:

```bash
python3 arm_driver/arm_driver/shared_control_param_sweep.py
```

Experiment analysis utilities are also exposed as ROS 2 console scripts:

```bash
ros2 run arm_driver shared_control_experiment_analyzer --help
ros2 run arm_driver shared_control_experiment_plotter --help
ros2 run arm_driver shared_control_batch_bag_recorder --help
ros2 run arm_driver shared_control_trial_logger --help
```

## Notes

- If FSR thresholds are changed in `hardware_adc`, restart both `hardware_adc` and `arm_control`.
- Before shutdown, publish `/arm_control_return_home` and wait for the arm to return to a safe pose.
- Do not commit `build/`, `install/`, `log/`, rosbag files, experiment outputs, or cache files.

## 中文补充

本项目是一个基于 ROS 2 Humble 的机械臂共享控制实验系统。人手操作摇杆，机械臂夹持摇杆，并通过虚拟安全边界对运动进行引导；当虚拟激光点越界后，系统切换为机器完全主导的反向回推。

核心结构：

- `arm_driver`：机械臂串口驱动、高层控制状态机、共享控制逻辑、Pygame 虚拟夹具。
- `sensor_interface`：ADS1115 采样、摇杆位置发布、FSR 接触/过压状态发布。

默认启动方式：

```bash
cd ~/arm_project_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm_driver system_launch.py
```

默认共享控制配置为 `arm_control_shared_balanced.yaml`。如果修改 FSR 接触或过压阈值，建议同时重启 `hardware_adc` 和 `arm_control`，避免传感器状态和控制节点参数不一致。
