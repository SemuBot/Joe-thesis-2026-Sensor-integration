# Joe-thesis-2026-Sensor-integration

**Bachelors Thesis**

---

## Overview
This repository contains all firmware and ROS 2 software developed for integrating ultrasonic proximity sensing into the SemuBot social humanoid robot platform.

The project includes:

- Implementing STM32 firmware for two different ultrasonic sensor types, each publishing distance measurements as `sensor_msgs/msg/Range` messages over micro-ROS
- Providing a ROS 2 navigation package (`ultrasonic_nav`) with reactive obstacle avoidance and joystick teleoperation nodes
- Automating the micro-ROS agent setup on a companion computer (Raspberry Pi) using Ansible

---

## Hardware Overview

| Parameter | Value |
|---|---|
| Microcontroller | STM32F303RE |
| Sensor variant A | HC-SR04 (PWM trigger/echo, up to 4 m range) |
| Sensor variant B | ICU-10201 (SPI, SonicLib, up to 1.2 m range) |
| Communication | UART (micro-ROS serial transport, 115 200 baud) |
| Programming tool | STM32CubeIDE |
| Computer | Raspberry Pi (ROS 2 Jazzy) |

Both firmware variants publish three `sensor_msgs/msg/Range` topics — `ultrasonic/left`, `ultrasonic/right`, and `ultrasonic/middle` — enabling the ROS 2 layer to treat the two sensor types interchangeably.

---

## Repository Structure

```
Joe-thesis-2026-Sensor-integration/
├── HC-SR04/
│   └── Ultrasonic_reader_/          # STM32CubeIDE project — HC-SR04 firmware
│       ├── Common/                  # Sensor driver (hcsr04.c/h, measurement.c/h)
│       ├── Core/                    # HAL init, main loop, micro-ROS publisher
│       ├── Drivers/                 # STM32F3xx HAL + CMSIS
│       ├── Middlewares/             # FreeRTOS source
│       ├── Ultrasonic_reader.ioc    # CubeMX pin/peripheral configuration
│       └── flash.sh                 # st-flash helper script
├── ICU-10201/
│   └── Ultrasonic_reader/           # STM32CubeIDE project — ICU-10201 firmware
│       ├── Common/                  # Sensor abstraction (sensor.c/h, obstacle_detection.c/h)
│       ├── Core/                    # HAL init, main loop, micro-ROS publisher
│       ├── Soniclib/                # TDK InvenSense SonicLib driver library
│       ├── bsp/                     # Board support package (chirp_stm32.c)
│       ├── Ultrasonic_reader.ioc    # CubeMX pin/peripheral configuration
│       └── flash.sh                 # st-flash helper script
├── semubot_ros2_collision_avoidance_ws/
│   └── src/
│       └── ultrasonic_nav/          # ROS 2 Python package
│           ├── ultrasonic_nav/
│           │   ├── obstacle_avoidance_node.py   # Reactive avoidance (state machine)
│           │   ├── bug2_node.py                 # Bug2 goal-directed navigation
│           │   ├── joy_vel.py                   # Joystick teleoperation with safety
│           │   └── distance_graph_node.py       # Real-time distance visualisation
│           ├── launch/
│           │   └── obstacle_avoidance.launch.py
│           └── config/
│               └── params.yaml
├── micro_ros_agent_setup.yml        # Ansible playbook — micro-ROS agent install
├── LICENSE
└── README.md
```

## Getting Started

### Prerequisites

- **STM32CubeIDE** (for firmware programming and building)
- **st-flash** / ST-Link programmer
- **ROS 2 Jazzy** on the main computer
- **Python 3** with 'rclpy', 'sensor_msgs', 'geometry_msgs', 'nav_msgs'
- **Ansible** (optional, for automated micro-ROS agent setup)

---

### 1. Clone the repositoriy
```git clone https://github.com/SemuBot/Joe-thesis-2026-Sensor-integration.git```

---

### 2. Set up micro-ROS agent on main Computer

Ansible playbook automates building and installing micro-ROS agent from 'https://github.com/micro-ROS/micro_ros_setup' on the Raspberry Pi running ROS2 Jazzy.

``` ansible-playbook micro_ros_agent_setup.yml ```

Micro-ROS agent alias is automatically added to '~/.bashrc'. To start the agent manually:

```source ~/micro-ros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 -b 115200 --reconnect```

---

### 3. Building and flashing firmware

Both sensor variants use the same build procedure.

**Using STM32CubeIDE**

1. Open STM32CubeIde and import the desired project as an existing project.
2. Build the project (**Project -> Build All**)
3. Flash via ST-Link (**Using the command line:** st-flash write build/Ultrasonic_reader.bin 0x08000000)

---

### 4. Build the ROS2 workspace

```cd semubot_ros2_collision_avoidance_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash```

---

### 5. Launch

The launch file start the micro-ROS serial agend and automatically start the obstacle_avoidance_node.
```ros2 launch ultrasonic_nav obstacle_avoidance.launch.py```

This launches:
- `micro_ros_agent`
- `obstacle_avoidance_node`

Optional_nodes:
- `bug2_node` - Goal oriented Bug2 navigation.
- `distance_graph_node` - real-time sensor distance plot.
- `joy_node` + `joy_vel` - joystick operation with safety override.

## License

See [LICENSE](LICENSE) for terms.


