# Phase 1: Foundation Setup - Detailed Implementation Plan

## Overview

**Phase:** 1 of 5
**Phase Name:** Foundation Setup
**Goal:** Establish the robot's basic hardware integration, sensor drivers, and visualization infrastructure

**Estimated Duration:** 3-5 days

---

## Objectives

By the end of Phase 1, the system should have:
1. Complete robot description (URDF) with all links and joints
2. Proper TF tree broadcasting all necessary transforms
3. micro-ROS agent running and communicating with chassis
4. RPLidar C1 driver publishing scan data
5. All sensor data visible in RViz2
6. Basic teleoperation capability

---

## Prerequisites

### Hardware
- [ ] Lunohod-2 differential drive chassis assembled
- [ ] micro-ROS firmware flashed on microcontroller
- [ ] RPLidar C1 connected via USB/Serial
- [ ] IMU integrated with chassis (micro-ROS)
- [ ] Power system functional

### Software
- [ ] ROS2 Kilted installed
- [ ] Workspace initialized at `/workspaces/ros-pg`
- [ ] lunohod-2 package created
- [ ] Dependencies installed (see section 8)

---

## Task Breakdown

### Task 1.1: Robot Description (URDF/Xacro)

**Objective:** Create complete robot model with all physical components

**Files to Create:**
- `description/lunohod2_core.xacro` - Main robot structure
- `description/lunohod2_lidar.xacro` - Lidar sensor model
- `description/lunohod2_gazebo.xacro` - Simulation plugins (optional)
- `description/lunohod2.urdf.xacro` - Master file that includes all modules

**Subtasks:**

#### 1.1.1 Create Base Robot Structure
**File:** `description/lunohod2_core.xacro`

**Requirements:**
- Define robot parameters (wheel radius, base width, etc.)
- Create `base_footprint` link (ground projection)
- Create `base_link` (robot center)
- Create wheel links (left/right)
- Define wheel joints (continuous, differential drive)
- Add collision and inertial properties
- Define visual meshes or basic geometry

**Key Parameters:**
```yaml
# Robot dimensions
wheel_radius: 0.065      # meters
wheel_width: 0.025       # meters
wheel_separation: 0.25   # meters (track width)
base_length: 0.35        # meters
base_width: 0.25         # meters
base_height: 0.15        # meters
ground_clearance: 0.05   # meters
```

**Validation:**
```bash
# Check URDF validity
check_urdf description/lunohod2.urdf.xacro

# Visualize robot structure
ros2 launch urdf_tutorial display.launch.py model:=src/lunohod-2/description/lunohod2.urdf.xacro
```

#### 1.1.2 Add Lidar Sensor
**File:** `description/lunohod2_lidar.xacro`

**Requirements:**
- Create `lidar_link`
- Position relative to `base_link`
- Add RPLidar C1 geometry (cylinder: r=0.038m, h=0.041m)
- Include sensor plugin for simulation (optional)

**RPLidar C1 Mounting:**
```yaml
# Typical front-center mounting
lidar_x: 0.10      # Forward from base_link
lidar_y: 0.0       # Centered
lidar_z: 0.12      # Above base_link
```

**Validation:**
- Lidar visible in RViz2
- `base_link` → `lidar_link` transform correct

#### 1.1.3 Add IMU Sensor
**File:** `description/lunohod2_core.xacro` (add to existing)

**Requirements:**
- Create `imu_link`
- Position at chassis center or actual IMU location
- No visual geometry needed (embedded sensor)

**IMU Mounting:**
```yaml
# Typically at base_link center or specific location
imu_x: 0.0
imu_y: 0.0
imu_z: 0.05
```

#### 1.1.4 Add Camera (Optional for Phase 1)
**File:** `description/lunohod2_camera.xacro`

**Requirements:**
- Create `camera_link`
- Position and orientation
- Camera plugin for simulation

**Note:** Camera is not used for autonomous navigation, only teleoperation/monitoring

---

### Task 1.2: Static Transform Publishers

**Objective:** Ensure all transforms are published correctly

**Files to Create:**
- `config/static_transforms.yaml` - Static TF parameters

**TF Tree Structure:**
```
map (published by SLAM/AMCL later)
 └─── odom (published by robot_localization in Phase 2)
       └─── base_footprint (published by micro-ROS or robot_state_publisher)
             └─── base_link (static transform)
                   ├─── lidar_link (static transform)
                   ├─── imu_link (static transform)
                   └─── camera_link (static transform, optional)
```

**Implementation:**
Two approaches:

**Option A: URDF-based (Recommended)**
- Define all links/joints in URDF
- Use `robot_state_publisher` to broadcast static transforms
- No separate static_transform_publisher needed

**Option B: Manual Static Transforms**
- Use `static_transform_publisher` nodes in launch file
- Useful if URDF incomplete or for quick testing

**Validation:**
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo base_link lidar_link

# Monitor all transforms
ros2 topic echo /tf_static
```

---

### Task 1.3: micro-ROS Agent Integration

**Objective:** Establish communication between micro-ROS chassis and ROS2 workspace

**Files to Create:**
- `config/micro_ros_agent.yaml` - Agent configuration
- `launch/micro_ros_agent.launch.py` - Agent launcher

**Subtasks:**

#### 1.3.1 Configure micro-ROS Agent

**Configuration File:** `config/micro_ros_agent.yaml`

**Parameters:**
```yaml
micro_ros_agent:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"  # Or /dev/ttyACM0
    baudrate: 115200             # Match firmware setting
    namespace: "ugv"             # Topic namespace
```

**Note:** Update baudrate and port based on actual hardware setup

#### 1.3.2 Create Agent Launch File

**File:** `launch/micro_ros_agent.launch.py`

**Launch Configuration:**
- Start micro-ROS agent with correct parameters
- Set up serial permissions if needed
- Include remappings if necessary

**Expected Topics:**
- `/ugv/odom` (nav_msgs/Odometry)
- `/ugv/imu` (sensor_msgs/Imu)
- `/ugv/encoder` (sensor_msgs/JointState)
- Subscribes to: `/ugv/cmd_vel` (geometry_msgs/Twist)

#### 1.3.3 Validate Communication

**Validation Steps:**
```bash
# 1. Start micro-ROS agent
ros2 launch lunohod-2 micro_ros_agent.launch.py

# 2. Check topics are published
ros2 topic list | grep ugv

# 3. Monitor odometry
ros2 topic echo /ugv/odom --once

# 4. Monitor IMU
ros2 topic echo /ugv/imu --once

# 5. Check publication rate
ros2 topic hz /ugv/odom  # Should be ~50 Hz

# 6. Send test velocity command
ros2 topic pub /ugv/cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

**Troubleshooting:**
- Check serial port permissions: `sudo chmod 666 /dev/ttyUSB0`
- Verify baudrate matches firmware
- Check USB connection stability
- Monitor for micro-ROS connection messages

---

### Task 1.4: RPLidar C1 Driver Setup

**Objective:** Get RPLidar C1 publishing laser scan data

**Files to Create:**
- `config/rplidar_c1.yaml` - Lidar parameters
- `launch/rplidar.launch.py` - Lidar launcher

**Subtasks:**

#### 1.4.1 Configure RPLidar Driver

**File:** `config/rplidar_c1.yaml`

**RPLidar C1 Specifications:**
- Range: 0.15m - 12m
- Scan rate: 10 Hz typical
- Angular resolution: ~0.9°
- Interface: USB (CP2102 serial)

**Configuration Parameters:**
```yaml
rplidar_node:
  ros__parameters:
    serial_port: "/dev/ttyUSB1"  # May differ from micro-ROS port
    serial_baudrate: 115200
    frame_id: "lidar_link"
    inverted: false
    angle_compensate: true
    scan_mode: "Standard"        # or "Sensitivity" for C1
    scan_frequency: 10.0         # Hz
```

**Note:** Check `sllidar_ros2` package documentation for C1-specific parameters

#### 1.4.2 Identify Serial Port

**Commands:**
```bash
# List USB devices
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*

# Check device info
udevadm info -a -n /dev/ttyUSB1 | grep -E 'ATTRS{idVendor}|ATTRS{idProduct}'

# RPLidar typically: idVendor=10c4, idProduct=ea60 (CP2102)
```

**Create udev Rule (Optional):**
```bash
# /etc/udev/rules.d/99-rplidar.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="rplidar"
```

Then use `/dev/rplidar` in config

#### 1.4.3 Create Lidar Launch File

**File:** `launch/rplidar.launch.py`

**Launch Configuration:**
- Start sllidar_node from sllidar_ros2 package
- Load parameters from config file
- Set up frame_id correctly

#### 1.4.4 Validate Lidar

**Validation:**
```bash
# 1. Start lidar
ros2 launch lunohod-2 rplidar.launch.py

# 2. Check scan topic
ros2 topic list | grep scan

# 3. Monitor scan data
ros2 topic echo /scan --once

# 4. Check scan rate
ros2 topic hz /scan  # Should be ~10 Hz

# 5. Visualize in RViz
ros2 run rviz2 rviz2
# Add LaserScan display, topic: /scan, frame: lidar_link
```

**Expected LaserScan Message:**
- `header.frame_id`: "lidar_link"
- `angle_min`: ~-3.14 rad
- `angle_max`: ~3.14 rad
- `angle_increment`: ~0.0157 rad (~0.9°)
- `range_min`: 0.15 m
- `range_max`: 12.0 m
- `ranges[]`: Array of distance measurements

---

### Task 1.5: Integrated Launch File

**Objective:** Single launch file to start all Phase 1 components

**File to Create:**
- `launch/bringup.launch.py` - Integrated system launcher

**Components to Launch:**
1. Robot state publisher (URDF)
2. Static transform publishers (if needed)
3. micro-ROS agent
4. RPLidar driver
5. RViz2 with custom config

**Launch File Structure:**
```python
# Pseudo-structure
- Load URDF from xacro
- Declare launch arguments (robot_name, use_sim, etc.)
- Node: robot_state_publisher
- Node: micro_ros_agent
- Node: rplidar_node
- Node: rviz2 (with config)
- Return LaunchDescription
```

**Launch Arguments:**
- `use_sim`: bool - Whether to launch in simulation mode
- `rviz_config`: path - Custom RViz configuration
- `serial_port_agent`: string - micro-ROS serial port
- `serial_port_lidar`: string - Lidar serial port

---

### Task 1.6: RViz Configuration

**Objective:** Create RViz config for visualizing Phase 1 components

**File to Create:**
- `config/phase1_visualization.rviz`

**RViz Displays to Include:**
1. **RobotModel**
   - Description Topic: `/robot_description`
   - TF Prefix: (empty)

2. **TF**
   - Show Names: True
   - Show Axes: True
   - Show Arrows: True
   - Marker Scale: 0.3

3. **LaserScan**
   - Topic: `/scan`
   - Size: 0.05
   - Color: By intensity or fixed
   - Decay Time: 0

4. **Odometry**
   - Topic: `/ugv/odom`
   - Keep: 100
   - Shape: Arrow
   - Color: By topic

5. **Imu** (optional)
   - Topic: `/ugv/imu`

**Global Options:**
- Fixed Frame: `odom` (or `base_link` if odom→base_footprint not yet published)

**Grid:**
- Reference Frame: `odom` or `base_link`
- Cell Size: 1.0 m

---

### Task 1.7: Basic Teleoperation Setup

**Objective:** Enable manual robot control for testing

**Options:**

#### Option A: Keyboard Teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ugv/cmd_vel
```

#### Option B: Joystick Teleop
```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

**Configuration File:** `config/teleop.yaml`
```yaml
teleop_twist_joy_node:
  ros__parameters:
    axis_linear: 1    # Left stick vertical
    axis_angular: 0   # Left stick horizontal
    scale_linear: 0.5
    scale_angular: 1.0
```

**Validation:**
- Robot responds to commands
- Movement matches joystick/keyboard input
- No lag or dropped commands

---

## Validation Checklist

### System Integration
- [ ] All ROS2 nodes start without errors
- [ ] No TF lookup errors in logs
- [ ] CPU usage acceptable (<50% on target platform)
- [ ] No memory leaks over 10-minute runtime

### Robot Description
- [ ] URDF passes `check_urdf` validation
- [ ] Robot model visible in RViz2
- [ ] All links present and correctly positioned
- [ ] Joints have proper types and limits

### TF Tree
- [ ] TF tree complete (`ros2 run tf2_tools view_frames`)
- [ ] No missing transforms
- [ ] Transform timestamps current (<1 sec old)
- [ ] Transform hierarchy correct (base_link → sensors)

### Sensor Data
- [ ] `/ugv/odom` publishing at ~50 Hz
- [ ] `/ugv/imu` publishing at ~50 Hz
- [ ] `/ugv/encoder` publishing at ~50 Hz
- [ ] `/scan` publishing at ~10 Hz
- [ ] All message headers have correct timestamps
- [ ] All message headers have correct frame_ids

### Lidar Quality
- [ ] Scan data shows environment geometry
- [ ] No excessive noise or invalid readings
- [ ] Range values within expected limits (0.15-12m)
- [ ] Angular coverage complete (360°)

### micro-ROS Communication
- [ ] Agent connects to microcontroller
- [ ] No reconnection loops
- [ ] Velocity commands received by chassis
- [ ] Odometry reflects actual movement

### Teleoperation
- [ ] Robot moves forward/backward
- [ ] Robot rotates left/right
- [ ] Movement speed controllable
- [ ] Emergency stop works (0 velocity)

### RViz Visualization
- [ ] Robot model renders correctly
- [ ] Lidar scans visible and accurate
- [ ] TF tree displays without errors
- [ ] Odometry trail visible
- [ ] Fixed frame stable (no jitter)

---

## Deliverables

### Code Files
1. `description/lunohod2_core.xacro`
2. `description/lunohod2_lidar.xacro`
3. `description/lunohod2.urdf.xacro`
4. `config/rplidar_c1.yaml`
5. `config/micro_ros_agent.yaml`
6. `config/phase1_visualization.rviz`
7. `launch/bringup.launch.py`
8. `launch/rplidar.launch.py`
9. `launch/micro_ros_agent.launch.py`

### Documentation
1. Hardware setup guide (serial ports, connections)
2. Troubleshooting guide (common issues)
3. Quick start commands
4. Parameter tuning notes

---

## Common Issues and Solutions

### Issue 1: Serial Port Permission Denied
**Symptoms:** `Permission denied: '/dev/ttyUSB0'`

**Solutions:**
```bash
# Temporary fix
sudo chmod 666 /dev/ttyUSB0

# Permanent fix - add user to dialout group
sudo usermod -a -G dialout $USER
# Then logout/login
```

### Issue 2: TF Transform Errors
**Symptoms:** `Could not transform from base_link to lidar_link`

**Solutions:**
- Check `robot_state_publisher` is running
- Verify URDF has all links defined
- Check frame_id names match exactly
- Monitor `/tf_static` topic

### Issue 3: Lidar Not Publishing
**Symptoms:** No `/scan` topic or empty scans

**Solutions:**
- Check serial port correct
- Verify lidar powered (should spin)
- Check baud rate (115200 for C1)
- Try different USB port/cable
- Check `dmesg` for USB errors

### Issue 4: micro-ROS Agent Disconnects
**Symptoms:** Frequent reconnection messages

**Solutions:**
- Check USB cable quality
- Verify baudrate matches firmware
- Check microcontroller power stability
- Monitor serial errors: `dmesg | tail`

### Issue 5: High CPU Usage
**Symptoms:** System slow, >80% CPU

**Solutions:**
- Reduce RViz display settings
- Lower lidar scan rate if possible
- Check for infinite loops in custom nodes
- Use `top` or `htop` to identify culprit

---

## Testing Procedure

### Test 1: Static System Test
**Duration:** 5 minutes

**Procedure:**
1. Launch all nodes: `ros2 launch lunohod-2 bringup.launch.py`
2. Open RViz with config
3. Verify all visualizations appear
4. Check `ros2 topic list` shows all expected topics
5. Monitor logs for errors

**Pass Criteria:** No errors, all topics present, RViz displays correct

---

### Test 2: Movement Test
**Duration:** 10 minutes

**Procedure:**
1. Start system with bringup launch
2. Start teleop keyboard
3. Drive robot forward 2m
4. Drive robot backward 2m
5. Rotate robot 360° clockwise
6. Rotate robot 360° counter-clockwise
7. Drive figure-8 pattern

**Pass Criteria:**
- Robot responds to all commands
- Movement smooth, no jerking
- Odometry updates during movement
- Lidar scans show environment changes

---

### Test 3: Sensor Data Quality Test
**Duration:** 15 minutes

**Procedure:**
1. Place robot in known environment
2. Record `/scan` data: `ros2 bag record /scan`
3. Record `/ugv/odom` data
4. Move robot through environment
5. Analyze bag file for:
   - Consistent scan rates
   - Reasonable range values
   - Continuous odometry updates

**Pass Criteria:**
- Scan rate 9-11 Hz
- <5% invalid readings
- Odometry rate 45-55 Hz
- No data gaps >200ms

---

### Test 4: Long Duration Test
**Duration:** 30 minutes

**Procedure:**
1. Start all nodes
2. Let system run idle for 15 minutes
3. Drive robot for 10 minutes
4. Let system run idle for 5 minutes
5. Check for memory leaks
6. Check for CPU degradation
7. Check for error accumulation

**Pass Criteria:**
- Memory usage stable (no leaks)
- CPU usage stable
- No accumulated errors in logs
- All topics still publishing

---

## Dependencies Installation

### Required ROS2 Packages
```bash
# Core packages
sudo apt install ros-kilted-robot-state-publisher
sudo apt install ros-kilted-joint-state-publisher
sudo apt install ros-kilted-xacro
sudo apt install ros-kilted-rviz2

# Teleop
sudo apt install ros-kilted-teleop-twist-keyboard
sudo apt install ros-kilted-teleop-twist-joy

# TF tools
sudo apt install ros-kilted-tf2-tools
sudo apt install ros-kilted-tf2-ros

# URDF tools
sudo apt install liburdfdom-tools

# Diagnostic tools
sudo apt install ros-kilted-rqt-robot-monitor
sudo apt install ros-kilted-rqt-graph
```

### Workspace Packages
```bash
# These should already be in workspace
# - micro-ROS-Agent (src/micro-ROS-Agent)
# - sllidar_ros2 (src/sllidar_ros2)

# Build workspace
cd /workspaces/ros-pg
colcon build --packages-select lunohod-2
source install/setup.bash
```

---

## Next Steps (Phase 2 Preview)

After completing Phase 1, you should have:
- Robot fully described and visualized
- All sensors publishing clean data
- Basic teleoperation working
- Solid foundation for advanced features

**Phase 2 will add:**
- Sensor fusion with robot_localization
- EKF filter configuration
- Fused odometry output
- Improved localization accuracy

---

## Success Criteria

Phase 1 is complete when:
1. ✅ All validation checklist items passed
2. ✅ All four tests passed successfully
3. ✅ Robot can be teleoperated reliably
4. ✅ All sensor data visible and accurate in RViz
5. ✅ No critical errors in 30-minute runtime
6. ✅ Documentation complete and accurate

---

**Document Version:** 1.0
**Created:** 2025-11-16
**Status:** Active Development
**Dependencies:** Main implementation plan (implementation_plan.md)
