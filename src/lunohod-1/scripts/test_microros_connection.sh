#!/bin/bash
# Test micro-ROS Connection Script
# Validates that the ESP32 micro-ROS node is publishing expected topics
# and that the robot is ready for navigation

set -e  # Exit on error

echo "=========================================="
echo "micro-ROS Connection Test for lunohod-1"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test 1: Check if micro-ROS topics exist
echo "1. Checking micro-ROS topics..."
echo "   Expected topics: /ugv/odom, /ugv/imu, /ugv/encoder, /cmd_vel"
echo ""

if ros2 topic list | grep -q "/ugv/odom"; then
    echo -e "${GREEN}✓${NC} /ugv/odom topic found"
else
    echo -e "${RED}✗${NC} /ugv/odom topic NOT found"
fi

if ros2 topic list | grep -q "/ugv/imu"; then
    echo -e "${GREEN}✓${NC} /ugv/imu topic found"
else
    echo -e "${RED}✗${NC} /ugv/imu topic NOT found"
fi

if ros2 topic list | grep -q "/ugv/encoder"; then
    echo -e "${GREEN}✓${NC} /ugv/encoder topic found"
else
    echo -e "${RED}✗${NC} /ugv/encoder topic NOT found"
fi

if ros2 topic list | grep -q "/cmd_vel"; then
    echo -e "${GREEN}✓${NC} /cmd_vel topic found"
else
    echo -e "${RED}✗${NC} /cmd_vel topic NOT found"
fi

echo ""

# Test 2: Check odometry publishing rate
echo "2. Checking /ugv/odom publishing rate..."
echo "   Expected: ~20-50 Hz"
echo ""
timeout 5 ros2 topic hz /ugv/odom || echo -e "${YELLOW}⚠${NC} Could not measure /ugv/odom rate (topic may not be publishing)"
echo ""

# Test 3: Check encoder data
echo "3. Checking /ugv/encoder data..."
echo "   Expected: Int32MultiArray with 2 values [left, right]"
echo ""
timeout 3 ros2 topic echo /ugv/encoder --once || echo -e "${YELLOW}⚠${NC} Could not read /ugv/encoder data"
echo ""

# Test 4: Check joint states conversion
echo "4. Checking /joint_states converter..."
if ros2 topic list | grep -q "/joint_states"; then
    echo -e "${GREEN}✓${NC} /joint_states topic found"
    echo "   Checking joint names..."
    timeout 3 ros2 topic echo /joint_states --once | grep -E "(left_wheel_joint|right_wheel_joint)" && \
        echo -e "${GREEN}✓${NC} Correct joint names detected" || \
        echo -e "${RED}✗${NC} Joint names may be incorrect"
else
    echo -e "${RED}✗${NC} /joint_states topic NOT found (encoder_to_joint_states node may not be running)"
fi
echo ""

# Test 5: Test motor control
echo "5. Testing motor control..."
echo "   Publishing test command: linear.x = 0.1 m/s for 1 second"
echo -e "${YELLOW}⚠${NC} Robot will move forward slightly if connected!"
read -p "   Press Enter to continue or Ctrl+C to skip..."
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --once && echo -e "${GREEN}✓${NC} Command sent successfully"
echo "   Stopping robot..."
sleep 1
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --once
echo ""

# Test 6: Check TF tree
echo "6. Checking TF tree (odom -> base_link)..."
echo "   Expected: Valid transform from odom to base_link"
echo ""
timeout 5 ros2 run tf2_ros tf2_echo odom base_link || \
    echo -e "${RED}✗${NC} Could not read odom -> base_link transform"
echo ""

# Test 7: List all active nodes
echo "7. Active ROS2 nodes:"
ros2 node list
echo ""

# Test 8: Check micro-ROS agent status
echo "8. Checking micro-ROS agent..."
if ros2 node list | grep -q "micro_ros_agent"; then
    echo -e "${GREEN}✓${NC} micro_ros_agent node is running"
else
    echo -e "${RED}✗${NC} micro_ros_agent node NOT running"
    echo "   Tip: Check if ESP32 is connected to /dev/ttyUSB0"
fi
echo ""

echo "=========================================="
echo "Test Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  - If all tests passed, try: ros2 launch lunohod-1 navigation_launch.py"
echo "  - To visualize: rviz2"
echo "  - To view TF tree: ros2 run tf2_tools view_frames"
echo ""
