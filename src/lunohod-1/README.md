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