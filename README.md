# Psyonic Hand ROS2 Control Package

ROS2 control driver for the [Psyonic Ability Hand](https://github.com/psyonicinc), compatible with MoveIt2 and real hardware, with built-in force sensing support. This package is part of the upcoming DexMobile project — a dual-arm mobile manipulation platform. Full system release coming soon.
<p align="center"> 
<img src="https://github.com/Marsenrage/Marsenrage.github.io/blob/master/images/psyonic_demo.gif" alt=psyonic demo" width="400"/>
</p>
<p> This project is funded by NSF #2420355</p>

## Features
- 🦾 6-DOF hand control (4 fingers + 2-DOF thumb)
- 📊 30 FSR force sensors
- 🎯 Trajectory following at 125Hz
- 🖥️ Mock hardware support

## System Requirements
- ✅ ROS2 Jazzy
- ✅ Ubuntu 24.04 LTS

## Quick Start

### Installation

```bash
mkdir -p ~/psyonic_ws/src && cd ~/psyonic_ws/src
git clone https://github.com/Marsenrage/psyonic_ros2_control.git
cd ~/psyonic_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build && source install/setup.bash
```

### Usage

**Simulation:**
```bash
ros2 launch psyonic_bring_up psyonic_bringup.launch.py use_mock_psyonic:=true
```

**Real Hardware:**
```bash
ros2 launch psyonic_bring_up psyonic_bringup.launch.py use_mock_psyonic:=false
```

**Test:**
```bash
python3 src/psyonic_ros2_control/psyonic_bring_up/scripts/test_psyonic_hand.py 
```

## Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_mock_psyonic` | `false` | Use simulation |
| `device_name` | `/dev/ttyUSB0` | Serial device |
| `baud_rate` | `460800` | Serial baud rate |

## API

**Topics:**
- `/left_hand/joint_states` - Joint positions/velocities/efforts
- `/psyonic_hand/fsr_sensors` - Force sensor data (30 values)

**Action:**
- `/left_hand/psyonic_left_hand/follow_joint_trajectory` - Send position commands


## Troubleshooting

**Serial permissions:**
```bash
sudo chmod 666 /dev/ttyUSB0
sudo usermod -a -G dialout $USER  # then logout/login
```

**Check controllers:**
```bash
ros2 control list_controllers
```

## License

Apache 2.0 License
