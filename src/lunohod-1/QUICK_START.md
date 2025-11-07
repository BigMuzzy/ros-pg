# lunohod-1 Quick Start Guide (micro-ROS)

## 🚀 Launch Commands

### Standard Launch
```bash
ros2 launch lunohod-1 launch_robot.launch.py
```

### Custom Device Paths
```bash
ros2 launch lunohod-1 launch_robot.launch.py \
  microros_device:=/dev/ttyUSB0 \
  lidar_port:=/dev/ttyUSB1
```

---

## ✅ Testing

### Quick Connection Test
```bash
cd /home/max/projects/ros-pg/src/lunohod-1
./scripts/test_microros_connection.sh
```

### Manual Checks
```bash
# Check topics
ros2 topic list | grep ugv

# Monitor odometry
ros2 topic echo /ugv/odom

# Test motors (robot will move!)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once

# Stop motors
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}}" --once

# Check TF
ros2 run tf2_ros tf2_echo odom base_link
```

---

## 🔧 Build

```bash
cd /home/max/projects/ros-pg
colcon build --packages-select lunohod-1
source install/setup.bash
```

---

## 📊 Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ugv/odom` | Odometry | ESP32 wheel odometry |
| `/ugv/encoder` | Int32MultiArray | Raw encoder [left, right] |
| `/ugv/imu` | Imu | ESP32 IMU data |
| `/cmd_vel` | Twist | Motor commands |
| `/joint_states` | JointState | Converted wheel positions |

---

## 🛠️ Troubleshooting

### ESP32 Not Connecting
```bash
# Check device
ls -l /dev/ttyUSB*

# Fix permissions
sudo chmod 666 /dev/ttyUSB0
```

### No Topics
```bash
# Restart micro-ROS agent
# Press Ctrl+C in launch terminal, then relaunch
ros2 launch lunohod-1 launch_robot.launch.py
```

### No Joint States
```bash
# Check encoder data
ros2 topic echo /ugv/encoder --once

# Check converter node
ros2 node list | grep encoder
```

---

## 📁 File Locations

- **Launch:** `launch/launch_robot.launch.py`
- **Config:** `config/nav2_params.yaml`, `config/twist_mux.yaml`
- **Scripts:** `scripts/encoder_to_joint_states.py`
- **Tests:** `scripts/test_microros_connection.sh`

---

## 🔗 Key Parameters

**Encoder Converter:**
- `wheel_radius`: 0.0325 m
- `counts_per_rev`: 5400

**Device Paths:**
- ESP32: `/dev/ttyUSB0` (default)
- Lidar: `/dev/ttyUSB1` (default)

---

For detailed information, see: `MICRO_ROS_MIGRATION_SUMMARY.md`
