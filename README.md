# Psyonic Hand ROS2 Control Package

ROS2 control package for the [Psyonic Ability Hand](https://github.com/psyonicinc) prosthetic with trajectory control and force sensing.

## Features

- 🦾 6-DOF hand control (4 fingers + 2-DOF thumb)
- 📊 30 FSR force sensors
- 🎯 Trajectory following at 125Hz
- 🖥️ Mock hardware support

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
python3 src/psyonic_bring_up/scripts/test_psyonic_hand.py
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
