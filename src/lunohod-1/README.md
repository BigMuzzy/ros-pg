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
ros2 launch lunohod-1 rsp.launch.py use_sim_time:=true
```

2. In a separate terminal, start the joint state publisher:
```bash
ros2 run joint_state_publisher joint_state_publisher
```

3. Launch RViz:
```bash
rviz2
```

4. In RViz:
   - Set **Fixed Frame** to `base_link`
   - Add a **RobotModel** display
   - The robot should appear with a white chassis, blue wheels, and black caster wheel

## Available Frames

- `base_link` (root frame)
- `chassis` (robot body)
- `left_wheel` / `right_wheel` (drive wheels)
- `caster_wheel` (support wheel)