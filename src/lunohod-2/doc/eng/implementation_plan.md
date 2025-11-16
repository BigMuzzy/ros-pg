# Lunohod-2 Implementation Plan

## Project Overview

**Robot Name:** Lunohod-2
**Platform:** Differential Drive Robot
**ROS2 Distribution:** Kilted
**Primary Navigation:** Nav2 Stack

### Mission Statement
Develop a robust autonomous mobile robot capable of mapping environments and navigating using pre-built maps, suitable for indoor autonomous navigation tasks.

---

## 1. Hardware Components

### 1.1 Chassis
- **Type:** Differential drive with micro-ROS controller
- **Update Rate:** 50 Hz
- **Capabilities:** Odometry, encoder feedback, IMU integration

### 1.2 Sensors
| Sensor | Model | Purpose | Topic |
|--------|-------|---------|-------|
| Lidar | RPLidar C1 | 2D environment scanning | `/scan` |
| IMU | Integrated | Orientation and acceleration | `/ugv/imu` |
| Odometry | Wheel encoders | Dead reckoning | `/ugv/odom` |
| Camera | Generic | Teleoperation/monitoring only | `/camera/image_raw` |
| Encoders | Integrated | Wheel rotation feedback | `/ugv/encoder` |

### 1.3 Actuators
- Differential drive motors (controlled via `/ugv/cmd_vel`)

---

## 2. System Architecture

### 2.1 Node Architecture Diagram
```
┌─────────────────────────────────────────────────────────────┐
│                    LUNOHOD-2 SYSTEM                          │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐         ┌──────────────┐                  │
│  │  micro-ROS   │         │   RPLidar    │                  │
│  │   Chassis    │         │   C1 Node    │                  │
│  └──────┬───────┘         └──────┬───────┘                  │
│         │                        │                           │
│         │ /ugv/odom             │ /scan                     │
│         │ /ugv/imu              │                           │
│         │ /ugv/encoder          │                           │
│         │                        │                           │
│         ▼                        ▼                           │
│  ┌──────────────────────────────────────┐                   │
│  │     Robot Localization (EKF)         │                   │
│  │  Fuses: odom + imu → /odometry/fused │                   │
│  └──────────────┬───────────────────────┘                   │
│                 │                                            │
│                 ▼                                            │
│  ┌─────────────────────────────────────────────┐            │
│  │          MODE SELECTION SWITCH              │            │
│  └─────┬──────────────────────────────────┬────┘            │
│        │                                   │                 │
│        ▼                                   ▼                 │
│  ┌──────────────┐                  ┌─────────────────┐      │
│  │ MAP BUILDING │                  │   NAVIGATION    │      │
│  │    MODE      │                  │      MODE       │      │
│  │              │                  │                 │      │
│  │ slam_toolbox │                  │  Nav2 Stack     │      │
│  │   (online)   │                  │   - AMCL        │      │
│  │              │                  │   - Planner     │      │
│  │ Outputs:     │                  │   - Controller  │      │
│  │  - /map      │                  │   - Recovery    │      │
│  │  - map.pgm   │                  │                 │      │
│  │  - map.yaml  │                  │ Input: map file │      │
│  └──────┬───────┘                  └────────┬────────┘      │
│         │                                   │                │
│         └───────────────┬───────────────────┘                │
│                         │                                    │
│                         ▼                                    │
│                  ┌─────────────┐                             │
│                  │ /ugv/cmd_vel│                             │
│                  └──────┬──────┘                             │
│                         │                                    │
│                         ▼                                    │
│                  ┌─────────────┐                             │
│                  │  Twist Mux  │                             │
│                  └──────┬──────┘                             │
│                         │                                    │
│                         ▼                                    │
│                  ┌──────────────┐                            │
│                  │ micro-ROS    │                            │
│                  │ Chassis      │                            │
│                  └──────────────┘                            │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 TF Tree Structure
```
map
 └─── odom
       └─── base_footprint
             └─── base_link
                   ├─── lidar_link
                   ├─── imu_link
                   └─── camera_link
```

---

## 3. Operation Modes

### 3.1 Mode 1: Map Building
**Purpose:** Create a 2D occupancy grid map of the environment using SLAM

**Active Components:**
- slam_toolbox (online synchronous mode)
- Robot localization (EKF fusion)
- RPLidar C1 driver
- micro-ROS chassis

**Workflow:**
1. Launch slam_toolbox with online mapping configuration
2. Robot teleoperated or autonomous exploration
3. Real-time map generation from lidar scans
4. Map saved as `.pgm` and `.yaml` files
5. Optionally serialize SLAM pose graph for later reuse

**Launch File:** `mapping_mode.launch.py`

**Key Topics:**
- Input: `/scan`, `/odometry/fused`
- Output: `/map`, `/map_metadata`

**User Operations:**
```bash
# Start mapping
ros2 launch lunohod-2 mapping_mode.launch.py

# Save map when complete
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_environment
```

---

### 3.2 Mode 2: Navigation
**Purpose:** Autonomous navigation using a pre-built map

**Active Components:**
- Nav2 navigation stack
- AMCL (localization)
- Global planner (NavFn or Smac)
- Local controller (DWB or TEB)
- Recovery behaviors
- Robot localization (EKF fusion)
- RPLidar C1 driver
- micro-ROS chassis

**Workflow:**
1. Load pre-saved map
2. Initialize AMCL for localization
3. Set navigation goal via RViz2 or API
4. Nav2 plans and executes path
5. Real-time obstacle avoidance
6. Recovery behaviors on failure

**Launch File:** `navigation_mode.launch.py`

**Key Topics:**
- Input: `/scan`, `/odometry/fused`, `/map`
- Output: `/cmd_vel`, `/plan`, `/local_plan`

**User Operations:**
```bash
# Start navigation
ros2 launch lunohod-2 navigation_mode.launch.py map:=/path/to/map.yaml

# Set goal via CLI
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}}"
```

---

## 4. Key Topics and Data Flow

### 4.1 Input Topics (from hardware)
| Topic | Type | Rate | Source |
|-------|------|------|--------|
| `/ugv/odom` | nav_msgs/Odometry | 50 Hz | micro-ROS chassis |
| `/ugv/imu` | sensor_msgs/Imu | 50 Hz | micro-ROS chassis |
| `/ugv/encoder` | sensor_msgs/JointState | 50 Hz | micro-ROS chassis |
| `/scan` | sensor_msgs/LaserScan | ~10 Hz | RPLidar C1 |

### 4.2 Processed Topics
| Topic | Type | Purpose |
|-------|------|---------|
| `/odometry/fused` | nav_msgs/Odometry | EKF-fused odometry (odom + IMU) |
| `/map` | nav_msgs/OccupancyGrid | Current map (SLAM or loaded) |
| `/tf` | tf2_msgs/TFMessage | Transform tree |
| `/tf_static` | tf2_msgs/TFMessage | Static transforms |

### 4.3 Output Topics (to hardware)
| Topic | Type | Purpose |
|-------|------|---------|
| `/ugv/cmd_vel` | geometry_msgs/Twist | Velocity commands to chassis |

---

## 5. Implementation Phases

### Phase 1: Foundation Setup
**Deliverables:**
- [ ] Package structure created
- [ ] URDF/Xacro robot description
- [ ] Static TF publishers configured
- [ ] micro-ROS agent integration tested
- [ ] RPLidar C1 driver configured and tested

**Validation:**
- Robot description visible in RViz2
- TF tree complete and accurate
- Sensor data publishing successfully

---

### Phase 2: Sensor Fusion
**Deliverables:**
- [ ] robot_localization EKF configuration
- [ ] Odometry + IMU fusion working
- [ ] `/odometry/fused` topic publishing
- [ ] Covariance tuning completed

**Validation:**
- Fused odometry more stable than raw odometry
- Orientation drift reduced by IMU integration

---

### Phase 3: Map Building Mode
**Deliverables:**
- [ ] slam_toolbox configuration file
- [ ] Mapping launch file
- [ ] Parameter tuning for RPLidar C1
- [ ] Map saving procedure documented
- [ ] RViz2 configuration for mapping

**Validation:**
- Consistent maps generated in test environment
- Loop closure detection working
- Maps saved successfully

---

### Phase 4: Navigation Mode
**Deliverables:**
- [ ] Nav2 parameter configurations
  - [ ] AMCL parameters
  - [ ] Global planner parameters
  - [ ] Local controller parameters
  - [ ] Costmap parameters (global and local)
  - [ ] Recovery behavior parameters
- [ ] Navigation launch file
- [ ] Map loading functionality
- [ ] RViz2 configuration for navigation

**Validation:**
- Robot localizes correctly on map
- Path planning successful
- Obstacle avoidance functional
- Goal reaching within tolerance

---

### Phase 5: Integration and Testing
**Deliverables:**
- [ ] Mode switching mechanism (separate launch files)
- [ ] Complete documentation
- [ ] Testing procedures
- [ ] Performance benchmarks
- [ ] Troubleshooting guide

**Validation:**
- End-to-end workflow tested
- Both modes work reliably
- Documentation complete

---

## 6. Configuration Files Required

### 6.1 Robot Description
- `description/lunohod2.urdf.xacro` - Main robot model
- `description/lunohod2_gazebo.xacro` - Simulation plugins (optional)

### 6.2 Sensor Fusion
- `config/ekf.yaml` - robot_localization EKF configuration

### 6.3 Lidar
- `config/rplidar_c1.yaml` - RPLidar C1 driver parameters

### 6.4 SLAM
- `config/slam_toolbox_online_sync.yaml` - SLAM configuration

### 6.5 Navigation
- `config/nav2_params.yaml` - Master Nav2 configuration
  - AMCL configuration
  - Planner configuration
  - Controller configuration
  - Global costmap configuration
  - Local costmap configuration
  - Recovery behavior configuration

### 6.6 RViz
- `config/mapping.rviz` - RViz config for mapping mode
- `config/navigation.rviz` - RViz config for navigation mode

---

## 7. Launch Files

### 7.1 Common Launch
- `launch/common.launch.py` - Shared components (robot description, TF, sensors)

### 7.2 Mode-Specific Launch
- `launch/mapping_mode.launch.py` - Map building mode
- `launch/navigation_mode.launch.py` - Navigation mode

---

## 8. Testing Strategy

### 8.1 Unit Testing
- [ ] TF tree integrity
- [ ] Topic publication rates
- [ ] Message type validation
- [ ] Parameter loading

### 8.2 Integration Testing
- [ ] Sensor fusion accuracy
- [ ] SLAM loop closure
- [ ] Navigation path planning
- [ ] Obstacle avoidance

### 8.3 Performance Testing
- [ ] CPU usage monitoring
- [ ] Memory consumption
- [ ] Real-time performance (50 Hz odometry)
- [ ] Map quality metrics

### 8.4 Field Testing
- [ ] Mapping in various environments
- [ ] Navigation in known environments
- [ ] Long-duration reliability tests
- [ ] Recovery from failures

---

## 9. Dependencies and Requirements

### 9.1 ROS2 Packages
```bash
# Core packages
sudo apt install ros-kilted-robot-state-publisher
sudo apt install ros-kilted-joint-state-publisher
sudo apt install ros-kilted-xacro
sudo apt install ros-kilted-rviz2

# Navigation
sudo apt install ros-kilted-nav2-bringup
sudo apt install ros-kilted-slam-toolbox
sudo apt install ros-kilted-robot-localization

# Micro-ROS
# (custom build from source - already in workspace)

# RPLidar
# (sllidar_ros2 - already in workspace)
```

### 9.2 Hardware Requirements
- Differential drive chassis with micro-ROS firmware
- RPLidar C1
- IMU (integrated with chassis)
- Sufficient compute (Raspberry Pi 4+ or similar)

---

## 10. Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Odometry drift | Use robot_localization EKF fusion with IMU |
| Lidar noise | Tune SLAM and costmap filter parameters |
| Navigation failures | Implement recovery behaviors, tune controller |
| Map quality issues | Slow mapping speed, multiple passes, parameter tuning |
| Real-time performance | Monitor CPU, optimize node configurations |

---

## 11. Future Enhancements (Out of Scope for Initial Implementation)

- 3D SLAM using camera (visual SLAM)
- Dynamic obstacle detection and tracking
- Multi-floor mapping
- Fleet management for multiple robots
- Autonomous exploration algorithm
- Web-based control interface
- Integration with external planning systems

---

## 12. References

- [Nav2 Documentation](https://navigation.ros.org/)
- [slam_toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization Documentation](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [RPLidar ROS2 Driver](https://github.com/Slamtec/sllidar_ros2)
- [micro-ROS Documentation](https://micro.ros.org/)

---

**Document Version:** 1.0
**Last Updated:** 2025-11-16
**Status:** Initial Draft
