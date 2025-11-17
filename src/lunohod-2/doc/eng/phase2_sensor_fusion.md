# Phase 2: Sensor Fusion - Detailed Implementation Plan

## Overview

**Phase:** 2 of 5
**Phase Name:** Sensor Fusion
**Goal:** Implement sensor fusion using robot_localization to combine wheel odometry and IMU data for improved pose estimation

**Estimated Duration:** 2-3 days

---

## Objectives

By the end of Phase 2, the system should have:
1. robot_localization package installed and configured
2. EKF (Extended Kalman Filter) node running
3. Wheel odometry and IMU data fused into single odometry estimate
4. odom→base_link transform published to TF tree
5. Improved pose estimation accuracy
6. Validated sensor fusion quality

---

## Prerequisites

### Phase 1 Complete
- [x] Robot description (URDF) complete
- [x] micro-ROS agent publishing /ugv/odom and /ugv/imu
- [x] RPLidar publishing /scan
- [x] All TF frames publishing correctly
- [x] Teleoperation working

### Current System State
**Topics Available:**
- `/ugv/odom` (nav_msgs/Odometry) - Wheel odometry at 50Hz
- `/ugv/imu` (sensor_msgs/Imu) - IMU data at 50Hz
- `/scan` (sensor_msgs/LaserScan) - Lidar at 10Hz

**TF Tree (Current):**
```
base_link
├─── chassis → sensors
└─── base_footprint
```

**TF Tree (After Phase 2):**
```
odom ← NEW (published by robot_localization)
└─── base_link
     ├─── chassis → sensors
     └─── base_footprint
```

---

## Task Breakdown

### Task 2.1: Install robot_localization

**Objective:** Install and verify robot_localization package

**Installation:**
```bash
sudo apt install ros-kilted-robot-localization
```

**Verification:**
```bash
# Check package installed
ros2 pkg list | grep robot_localization

# Check ekf_node available
ros2 pkg executables robot_localization

# Expected output:
# robot_localization ekf_node
# robot_localization navsat_transform_node
# robot_localization ukf_node
```

---

### Task 2.2: Analyze Current Sensor Data

**Objective:** Understand sensor characteristics before fusion

**Subtasks:**

#### 2.2.1 Analyze Wheel Odometry

**Commands:**
```bash
# Terminal 1: Start robot
ros2 launch lunohod-2 bringup.launch.py

# Terminal 2: Inspect odometry message
ros2 topic echo /ugv/odom --once

# Check publication rate
ros2 topic hz /ugv/odom

# Record odometry for analysis
ros2 bag record /ugv/odom -o odom_test --duration 30
```

**What to Check:**
- **Frame IDs:** header.frame_id should be "odom", child_frame_id should be "base_link"
- **Covariance:** Check if covariance values are populated (non-zero)
- **Position:** x, y, z values
- **Orientation:** Quaternion values (should change when rotating)
- **Velocities:** Linear and angular velocities

**Expected Issues:**
- Frame IDs might need remapping if different
- Covariance might be all zeros (needs configuration)
- Orientation might not update (wheel odometry typically doesn't provide yaw rate)

#### 2.2.2 Analyze IMU Data

**Commands:**
```bash
# Inspect IMU message
ros2 topic echo /ugv/imu --once

# Check publication rate
ros2 topic hz /ugv/imu

# Record IMU for analysis
ros2 bag record /ugv/imu -o imu_test --duration 30
```

**What to Check:**
- **Frame ID:** header.frame_id should be "imu_link"
- **Orientation:** Quaternion (fused from accel/gyro on micro-ROS side)
- **Angular Velocity:** omega_x, omega_y, omega_z (gyroscope)
- **Linear Acceleration:** accel_x, accel_y, accel_z (accelerometer)
- **Covariances:** Check if populated

**IMU Data Validity Test:**
```bash
# While robot stationary, IMU should show:
# - Linear acceleration: ~0, 0, ~9.81 (gravity in z-axis)
# - Angular velocity: ~0, ~0, ~0
# - Orientation: stable quaternion

# While robot rotating, IMU should show:
# - Angular velocity z changes
# - Orientation changes
```

---

### Task 2.3: Create EKF Configuration

**Objective:** Configure robot_localization EKF for sensor fusion

**File to Create:** `config/ekf.yaml`

#### 2.3.1 Basic EKF Configuration

**Configuration Strategy:**
- Fuse wheel odometry (X, Y, yaw velocities)
- Fuse IMU (orientation, angular velocity, linear acceleration)
- Output fused odometry on /odometry/filtered
- Publish odom→base_link transform

**Key Parameters:**

**Frame Configuration:**
```yaml
ekf_filter_node:
  ros__parameters:
    # Frequency of filter prediction/correction (Hz)
    frequency: 50.0

    # Sensor timeout (seconds)
    sensor_timeout: 0.1

    # Frame names
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # Transform publishing
    publish_tf: true
    publish_acceleration: false
```

**Wheel Odometry Configuration:**
```yaml
    # Odometry sensor
    odom0: /ugv/odom
    odom0_config: [false, false, false,    # x, y, z position
                   false, false, false,    # roll, pitch, yaw orientation
                   true,  true,  false,    # x_dot, y_dot, z_dot velocity
                   false, false, true,     # roll_dot, pitch_dot, yaw_dot
                   false, false, false]    # x_ddot, y_ddot, z_ddot acceleration

    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10
```

**IMU Configuration:**
```yaml
    # IMU sensor
    imu0: /ugv/imu
    imu0_config: [false, false, false,     # x, y, z position
                  true,  true,  true,      # roll, pitch, yaw orientation
                  false, false, false,     # x_dot, y_dot, z_dot velocity
                  false, false, true,      # roll_dot, pitch_dot, yaw_dot
                  true,  true,  true]      # x_ddot, y_ddot, z_ddot acceleration

    imu0_differential: false
    imu0_relative: false
    imu0_queue_size: 10
    imu0_remove_gravitational_acceleration: true
```

**Process Noise Covariance:**
```yaml
    # Process noise covariance (Q matrix)
    # Higher values = trust process model less
    process_noise_covariance: [0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015]
```

**Initial State Covariance:**
```yaml
    # Initial state covariance (P matrix)
    initial_estimate_covariance: [1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                   0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                   0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                   0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                   0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                   0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,
                                   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,
                                   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,
                                   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,
                                   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,
                                   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9]
```

#### 2.3.2 Understanding EKF State Vector

The EKF state vector has 15 elements:
```
[0-2]:   Position (x, y, z)
[3-5]:   Orientation (roll, pitch, yaw)
[6-8]:   Linear velocity (vx, vy, vz)
[9-11]:  Angular velocity (vroll, vpitch, vyaw)
[12-14]: Linear acceleration (ax, ay, az)
```

**Sensor Configuration Strategy:**

**Wheel Odometry (/ugv/odom):**
- Use: Linear velocities (vx, vy), angular velocity (vyaw)
- Don't use: Position (drifts), orientation (unreliable from encoders)

**IMU (/ugv/imu):**
- Use: Orientation (roll, pitch, yaw), angular velocity (vyaw), linear acceleration
- Don't use: Position (IMU doesn't provide position)

---

### Task 2.4: Create EKF Launch File

**Objective:** Launch robot_localization EKF node with configuration

**File to Create:** `launch/ekf.launch.py`

**Launch File Content:**
```python
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_lunohod2 = get_package_share_directory('lunohod-2')

    # EKF configuration file
    ekf_config = os.path.join(pkg_lunohod2, 'config', 'ekf.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_node,
    ])
```

---

### Task 2.5: Integrate EKF into Bringup

**Objective:** Add EKF to main bringup launch file

**File to Modify:** `launch/bringup.launch.py`

**Add Launch Argument:**
```python
use_ekf_arg = DeclareLaunchArgument(
    'use_ekf',
    default_value='false',
    description='Launch EKF for sensor fusion'
)
```

**Add EKF Node:**
```python
# EKF configuration file
ekf_config = os.path.join(pkg_lunohod2, 'config', 'ekf.yaml')

# EKF node (conditional)
ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    condition=IfCondition(use_ekf)
)
```

**Add to LaunchDescription:**
```python
return LaunchDescription([
    # ... existing arguments ...
    use_ekf_arg,

    # ... existing nodes ...
    ekf_node,
])
```

---

### Task 2.6: Test EKF Launch

**Objective:** Verify EKF starts without errors

**Test Procedure:**

**Step 1: Launch with EKF**
```bash
ros2 launch lunohod-2 bringup.launch.py use_ekf:=true
```

**Step 2: Check EKF node running**
```bash
ros2 node list | grep ekf
# Expected: /ekf_filter_node
```

**Step 3: Check filtered odometry topic**
```bash
ros2 topic list | grep odometry
# Expected: /odometry/filtered
```

**Step 4: Inspect filtered odometry**
```bash
ros2 topic echo /odometry/filtered --once
```

**Expected Output:**
- header.frame_id: "odom"
- child_frame_id: "base_link"
- Pose and twist populated with fused estimates

**Step 5: Check TF tree**
```bash
ros2 run tf2_ros tf2_echo odom base_link
```

**Expected Output:**
- Transform should be published
- Translation and rotation values present
- No errors about missing frames

**Step 6: Visualize TF tree**
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf showing odom → base_link transform
```

**Common Errors and Solutions:**

**Error: "Timed out waiting for transform"**
- Check publish_tf: true in ekf.yaml
- Verify sensor topics publishing
- Check frame_id names match exactly

**Error: "Could not find a connection between 'odom' and 'base_link'"**
- EKF not publishing transform
- Check sensor data arriving (ros2 topic hz /ugv/odom /ugv/imu)
- Check EKF logs for sensor timeouts

**Error: "Covariance specified for measurement is zero"**
- Sensor covariances all zero
- May need to set minimum covariance in EKF config

---

### Task 2.7: Validate Sensor Fusion Quality

**Objective:** Test and tune EKF performance

#### 2.7.1 Static Test

**Robot stationary, no movement:**

```bash
# Monitor filtered odometry
ros2 topic echo /odometry/filtered

# Expected behavior:
# - Position: stable (minimal drift)
# - Orientation: stable
# - Velocities: near zero
# - Accelerations: near zero
```

**Validation:**
- Position drift < 1cm over 1 minute
- Orientation drift < 1 degree over 1 minute
- Velocity noise < 0.01 m/s

#### 2.7.2 Linear Motion Test

**Drive robot forward in straight line:**

```bash
# Terminal 1: Launch with EKF and RViz
ros2 launch lunohod-2 bringup.launch.py use_ekf:=true use_rviz:=true

# Terminal 2: Keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ugv/cmd_vel

# Drive forward at constant speed for 2 meters
# Stop and observe
```

**Validation:**
- Odometry path appears straight in RViz
- Filtered odometry smoother than raw odometry
- No large jumps in position estimate

**Comparison Test:**
```bash
# Compare raw vs filtered odometry
ros2 topic echo /ugv/odom --field pose.pose.position
ros2 topic echo /odometry/filtered --field pose.pose.position
```

#### 2.7.3 Rotation Test

**Rotate robot 360 degrees:**

```bash
# Rotate slowly in place
# Complete full rotation
# Return to start orientation
```

**Validation:**
- Final orientation close to starting orientation (within 5 degrees)
- Rotation smooth in RViz
- IMU helps reduce yaw drift

#### 2.7.4 Combined Motion Test

**Drive figure-8 pattern:**

```bash
# Mix of forward motion and rotation
# Observe odometry trail in RViz
```

**Validation:**
- Path appears smooth
- No discontinuities or jumps
- Fused estimate more stable than raw odometry

---

### Task 2.8: Update RViz Configuration

**Objective:** Update RViz to use fused odometry and odom frame

**Changes to make in robot_view.rviz:**

**1. Change Fixed Frame:**
```yaml
Global Options:
  Fixed Frame: odom  # Changed from base_link
```

**2. Add Filtered Odometry Display:**
- Add new PoseWithCovariance display
- Topic: /odometry/filtered
- Show covariance ellipse

**3. Add Odometry Path Display:**
- Add Path display
- Topic: /odometry/filtered (needs path conversion)
- Or use Odometry display with history

**4. Compare Raw vs Filtered:**
- Display both /ugv/odom and /odometry/filtered
- Different colors to compare

---

### Task 2.9: Tuning EKF Parameters

**Objective:** Optimize EKF performance for your robot

#### Common Tuning Parameters:

**If odometry drifts too much:**
```yaml
# Increase process noise for position
process_noise_covariance[0,0]: 0.1  # x position
process_noise_covariance[1,1]: 0.1  # y position
```

**If estimate too jittery:**
```yaml
# Decrease process noise
# Increase sensor trust by using more measurements
```

**If IMU orientation not helping:**
```yaml
# Check IMU data quality
# Verify remove_gravitational_acceleration is correct
# Check IMU frame is correct in URDF
```

**If rotation drift:**
```yaml
# Increase IMU angular velocity weight
imu0_config: [..., true]  # Enable yaw_dot from IMU
```

---

## Validation Checklist

### System Integration
- [ ] EKF node starts without errors
- [ ] All sensor data received by EKF (check logs)
- [ ] Filtered odometry publishing at expected rate (~50 Hz)
- [ ] odom→base_link transform publishing
- [ ] No TF errors in system logs

### TF Tree
- [ ] TF tree includes odom frame
- [ ] odom→base_link transform present
- [ ] No transform lookup errors
- [ ] Transform timestamps current (<100ms old)

### Data Quality
- [ ] Filtered odometry smoother than raw odometry
- [ ] Static test: minimal drift (<1cm/min)
- [ ] Linear motion: straight paths appear straight
- [ ] Rotation: final orientation close to initial
- [ ] No large discontinuities in estimates

### RViz Visualization
- [ ] Fixed frame set to "odom"
- [ ] Robot model visible and correct
- [ ] Odometry trail visible (after movement)
- [ ] No transform errors in RViz
- [ ] Covariance ellipses reasonable size

---

## Deliverables

### Configuration Files
1. `config/ekf.yaml` - EKF filter configuration
2. `launch/ekf.launch.py` - Standalone EKF launcher
3. Updated `launch/bringup.launch.py` - Integrated EKF launch
4. Updated `config/robot_view.rviz` - Odom frame visualization

### Documentation
1. Sensor fusion testing results
2. Tuning notes and final parameters
3. Known issues and limitations

---

## Testing Procedure

### Test 1: EKF Integration Test
**Duration:** 5 minutes

**Procedure:**
1. Launch full system with EKF
2. Verify all nodes running
3. Check topic rates
4. Verify TF tree complete
5. Monitor for errors

**Pass Criteria:**
- No errors in launch
- /odometry/filtered publishing
- TF tree has odom frame

---

### Test 2: Static Stability Test
**Duration:** 5 minutes

**Procedure:**
1. Start system, robot stationary
2. Record /odometry/filtered
3. Analyze position drift
4. Analyze orientation drift

**Pass Criteria:**
- Position drift < 5cm over 5 minutes
- Orientation drift < 2 degrees over 5 minutes

---

### Test 3: Motion Accuracy Test
**Duration:** 10 minutes

**Procedure:**
1. Mark starting position
2. Drive forward 2m, stop
3. Drive backward 2m, stop (return to start)
4. Measure actual vs estimated position

**Pass Criteria:**
- Final position error < 10cm
- Path appears straight in RViz

---

### Test 4: Long Duration Test
**Duration:** 15 minutes

**Procedure:**
1. Start system with EKF
2. Drive robot through varied motions
3. Monitor CPU usage
4. Check for memory leaks
5. Verify continuous operation

**Pass Criteria:**
- CPU usage stable (<40%)
- Memory usage stable
- No accumulated errors
- Filter continues operating

---

## Common Issues and Solutions

### Issue 1: EKF Not Publishing Transform

**Symptoms:** No odom→base_link transform

**Solutions:**
```bash
# Check publish_tf setting
grep "publish_tf" config/ekf.yaml

# Check sensor data arriving
ros2 topic hz /ugv/odom /ugv/imu

# Check EKF logs
ros2 node info /ekf_filter_node
```

---

### Issue 2: Frame ID Mismatch

**Symptoms:** "Frame [X] does not exist"

**Solutions:**
```bash
# Check frame IDs in sensor data
ros2 topic echo /ugv/odom --field header.frame_id
ros2 topic echo /ugv/imu --field header.frame_id

# Verify URDF frames match configuration
ros2 run tf2_ros tf2_echo base_link imu_link
```

---

### Issue 3: Poor Fusion Quality

**Symptoms:** Filtered odometry not better than raw

**Solutions:**
1. Check sensor configurations (which states to fuse)
2. Verify covariances are reasonable
3. Tune process noise
4. Check sensor synchronization

---

### Issue 4: High CPU Usage

**Symptoms:** System slow, >80% CPU

**Solutions:**
1. Reduce EKF frequency (default 50Hz)
2. Reduce sensor queue sizes
3. Disable unnecessary state variables

---

## Next Steps (Phase 3 Preview)

After completing Phase 2:
- Odometry more accurate and stable
- Ready for SLAM mapping
- TF tree complete for navigation

**Phase 3 will add:**
- slam_toolbox for 2D mapping
- Map building mode
- Map saving/loading
- Ready for autonomous navigation

---

## Success Criteria

Phase 2 is complete when:
1. ✅ EKF running without errors
2. ✅ Sensor fusion working (odometry + IMU)
3. ✅ odom→base_link transform publishing
4. ✅ Filtered odometry smoother than raw
5. ✅ All validation tests passed
6. ✅ RViz shows complete TF tree with odom frame
7. ✅ Documentation complete

---

**Document Version:** 1.0
**Created:** 2025-11-17
**Status:** Active Development
**Dependencies:** phase1_foundation_setup.md, implementation_plan.md
