# Lunohod-1 Robot

## Build and Setup

```bash
# Build the package
colcon build --packages-select lunohod-1
source install/setup.bash
```

## Visualization in RViz

To visualize the robot in RViz:

1. Launch the robot state publisher:
```bash
ros2 launch lunohod-1 rsp.launch.py (use_sim_time:=true only if using Gazebo)
```
2. In RViz:
   - Set **Fixed Frame** to `base_link`
   - Add a **RobotModel** display
   - The robot should appear with a white chassis, blue wheels, and black caster wheel

## Available Frames

- `base_link` (root frame)
- `chassis` (robot body)
- `left_wheel` / `right_wheel` (drive wheels)
- `caster_wheel` (support wheel)

## Verification Steps

### Test URDF compilation:
```bash
cd ~/your_workspace
xacro src/lunohod-1/description/robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

### Visualize the tree:
```bash
urdf_to_graphiz /tmp/robot.urdf
```

Use [GraphvizOnline](https://dreampuf.github.io/GraphvizOnline/) to visualize the generated graph.

## Hardware Robot Launch Instructions

### Hardware Connections Expected

Your robot setup expects these device connections:

1. **Arduino/Motor Controller**: `/dev/ttyUSB0` (from `ros2_control.xacro`)
2. **RPLidar**: `/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0` (from `rplidar.launch.py`)
3. **Camera**: Should be detected as a V4L2 device (typically `/dev/video0` or similar)

### Launch Commands

#### 1. Main Robot Launch
```bash
# Build and source first
colcon build --packages-select lunohod-1
source install/setup.bash

# Launch the main robot
ros2 launch lunohod-1 launch_robot.launch.py
```

#### 2. Camera Launch (separate terminal)
```bash
source install/setup.bash
ros2 launch lunohod-1 camera.launch.py
```

#### 3. Lidar Launch (separate terminal)
```bash
source install/setup.bash
ros2 launch lunohod-1 rplidar.launch.py
```

#### 4. Visualization (optional - separate terminal)
```bash
source install/setup.bash
rviz2 -d src/lunohod-1/config/main.rviz
```

### Device Setup & Troubleshooting

#### Arduino/Motor Controller
- **Expected**: `/dev/ttyUSB0`
- **Check**: `ls /dev/ttyUSB*`
- **Permissions**: `sudo chmod 666 /dev/ttyUSB0` or add user to dialout group: `sudo usermod -a -G dialout $USER`

#### RPLidar
- **Expected**: `/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0`
- **Check actual path**: `ls /dev/serial/by-path/`
- **Update if different**: Edit `src/lunohod-1/launch/rplidar.launch.py` and change the `serial_port` parameter
- **Alternative**: Use `/dev/ttyUSB1` (or appropriate number) if the by-path doesn't work

#### Camera
- **Check**: `ls /dev/video*`
- **Test**: `v4l2-ctl --list-devices`
- The camera launch uses V4L2, so any USB camera should work automatically

### Quick Device Check Script
```bash
#!/bin/bash
echo "=== Device Check ==="
echo "USB Serial devices:"
ls /dev/ttyUSB* 2>/dev/null || echo "No ttyUSB devices found"

echo -e "\nSerial by-path:"
ls /dev/serial/by-path/ 2>/dev/null || echo "No serial by-path devices found"

echo -e "\nVideo devices:"
ls /dev/video* 2>/dev/null || echo "No video devices found"

echo -e "\nUSB devices:"
lsusb | grep -E "(Arduino|CP210|CH340|FTDI|Slamtec|RPLidar)"
```

### If Devices Are on Different Paths

#### For RPLidar
Edit `src/lunohod-1/launch/rplidar.launch.py` and change:
```python
'serial_port': '/dev/ttyUSB1',  # or whatever your actual port is
```

#### For Arduino
Edit `src/lunohod-1/description/ros2_control.xacro` and change:
```xml
<param name="device">/dev/ttyUSB1</param>  <!-- or your actual port -->
```

### Expected Topics After Launch

You should see these topics:
- `/scan` (from lidar)
- `/camera/image_raw` (from camera)
- `/cmd_vel` (for robot control)
- `/odom` (odometry from wheels)
- `/tf` and `/tf_static` (transforms)

Check with: `ros2 topic list`

The robot should now be ready for teleoperation, navigation, or SLAM!