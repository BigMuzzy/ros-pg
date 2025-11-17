# Lunohod-2 Teleoperation Guide

## Overview

The Lunohod-2 robot supports two teleoperation modes:
1. **Keyboard Control** - For testing and development
2. **Joystick Control** - For manual driving with Xbox/PS controller

## Prerequisites

### For Keyboard Control
```bash
sudo apt install ros-kilted-teleop-twist-keyboard
```

### For Joystick Control
```bash
sudo apt install ros-kilted-teleop-twist-joy
sudo apt install ros-kilted-joy
```

## Quick Start

**Step 1: Start the robot system**
```bash
# Terminal 1: Start all robot components
ros2 launch lunohod-2 bringup.launch.py
```

**Step 2: Start teleoperation**

### Keyboard Control (Recommended for Testing)

```bash
# Terminal 2: Keyboard teleop (run directly, NOT through launch file)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ugv/cmd_vel
```

**Note:** Keyboard teleop must be run directly in a terminal because it requires interactive keyboard input.

### Joystick Control

```bash
# Terminal 2: Joystick teleop
ros2 launch lunohod-2 teleop.launch.py
```

## Keyboard Controls

When using keyboard teleoperation, the following keys control the robot:

```
   u    i    o
   j    k    l
   m    ,    .

u/o : increase/decrease only linear speed
i/, : increase/decrease only angular speed
j/l : turn left/right
k   : force stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease linear speed
a/d : increase/decrease angular speed
s   : force all velocities to zero
```

**Safety:**
- Press `k` or `s` to immediately stop the robot
- Release keys and the robot will continue moving (use `k` to stop)
- Use `q/z` to adjust maximum speed limits

## Joystick Controls

### Xbox Controller Layout

- **Left Stick Vertical** - Forward/Backward movement
- **Left Stick Horizontal** - Rotation (left/right)
- **LB (Left Bumper)** - Enable button (must hold to move)
- **RB (Right Bumper)** - Turbo mode (faster speeds)

**Safety:**
- You must hold the **LB button** for the robot to respond to joystick commands
- Release LB to immediately stop the robot
- Turbo mode (RB) increases max speeds to 1.0 m/s linear and 1.5 rad/s angular

### Configuration

Edit joystick settings in: `config/teleop.yaml`

```yaml
# Example customization
scale_linear:
  x: 0.7  # Change max linear velocity

scale_angular:
  yaw: 1.2  # Change max angular velocity

enable_button: 4  # Change enable button (LB = 4)
```

## Launch File Parameters

The joystick teleop launch file supports the following parameters:

```bash
# Default launch (joystick on /dev/input/js0)
ros2 launch lunohod-2 teleop.launch.py

# Custom cmd_vel topic (if not using /ugv/cmd_vel)
ros2 launch lunohod-2 teleop.launch.py cmd_vel_topic:=/custom_cmd_vel

# Custom joystick device
ros2 launch lunohod-2 teleop.launch.py joy_device:=/dev/input/js1
```

**Note:** Velocity limits are configured in `config/teleop.yaml`, not as launch parameters.

## Troubleshooting

### Keyboard Teleop Issues

**Problem:** Terminal says "command not found"
```bash
# Solution: Install teleop_twist_keyboard
sudo apt install ros-kilted-teleop-twist-keyboard
source /opt/ros/kilted/setup.bash
```

**Problem:** Robot doesn't move
```bash
# Check if /ugv/cmd_vel topic is being published
ros2 topic echo /ugv/cmd_vel

# Check if micro-ROS agent is running
ros2 node list | grep micro_ros_agent
```

### Joystick Teleop Issues

**Problem:** Joystick not detected
```bash
# Check joystick device
ls -l /dev/input/js*

# Test joystick manually
sudo apt install joystick
jstest /dev/input/js0
```

**Problem:** Robot doesn't respond to joystick
```bash
# Make sure you're holding the enable button (LB)

# Check joy topic is publishing
ros2 topic echo /joy --once

# Verify cmd_vel is being published
ros2 topic echo /ugv/cmd_vel
```

**Problem:** Wrong button mappings
```bash
# Use jstest to find button numbers
jstest /dev/input/js0

# Update config/teleop.yaml with correct button IDs
```

### Permission Issues

```bash
# Add user to input group (for joystick)
sudo usermod -a -G input $USER

# Add user to dialout group (for serial)
sudo usermod -a -G dialout $USER

# Logout and login for changes to take effect
```

## Velocity Limits

### Default Limits
- **Linear Velocity:** 0.5 m/s (normal), 1.0 m/s (turbo)
- **Angular Velocity:** 1.0 rad/s (normal), 1.5 rad/s (turbo)

### Safety Recommendations
- Start with low speeds (0.3 m/s) when testing
- Increase speeds gradually after verifying robot behavior
- Always test emergency stop before full operation
- Use turbo mode only in open areas

## Integration with Main System

### Full System Launch

The bringup launch file can optionally include teleoperation:

```bash
# This is a potential future feature
# Not implemented in current version
ros2 launch lunohod-2 bringup.launch.py use_teleop:=true
```

### Monitoring Teleoperation

```bash
# Monitor velocity commands
ros2 topic echo /ugv/cmd_vel

# Check command frequency
ros2 topic hz /ugv/cmd_vel

# Visualize in RViz
ros2 launch lunohod-2 bringup.launch.py use_rviz:=true
```

## Testing Procedure

### Basic Functionality Test

1. **Start the robot:**
   ```bash
   ros2 launch lunohod-2 bringup.launch.py
   ```

2. **Start teleop:**
   ```bash
   ros2 launch lunohod-2 teleop.launch.py
   ```

3. **Test movements:**
   - Forward: Press `i`
   - Backward: Press `,`
   - Rotate left: Press `j`
   - Rotate right: Press `l`
   - Stop: Press `k`

4. **Verify response:**
   - Robot should move in expected direction
   - Movement should be smooth
   - Stop should be immediate

### Emergency Stop Test

1. Drive robot forward at full speed
2. Press emergency stop (`k` or release LB button)
3. Robot should stop within 0.5 seconds
4. Repeat test in all directions

## Next Steps

After verifying teleoperation works:
- **Phase 2:** Sensor Fusion (robot_localization)
- **Phase 3:** Map Building (slam_toolbox)
- **Phase 4:** Autonomous Navigation (Nav2)

## Related Documentation

- [Phase 1 Implementation](phase1_foundation_setup.md)
- [Main Implementation Plan](implementation_plan.md)
- [Bringup Launch File](../../launch/bringup.launch.py)
- [Teleop Launch File](../../launch/teleop.launch.py)

---

**Document Version:** 1.0
**Last Updated:** 2025-11-17
**Task:** Phase 1, Task 1.7 - Basic Teleoperation Setup
