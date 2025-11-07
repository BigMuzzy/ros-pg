#!/bin/bash
# Visualize TF Frame Tree Script
# Generates a PDF visualization of the robot's transform tree

set -e

echo "=========================================="
echo "TF Frame Visualization for lunohod-1"
echo "=========================================="
echo ""

# Check if evince (PDF viewer) is installed
if ! command -v evince &> /dev/null; then
    VIEWER="xdg-open"
    echo "Note: evince not found, will use xdg-open to view PDF"
else
    VIEWER="evince"
fi

# Generate the frame tree
echo "Generating TF frame tree..."
echo "This will create a file called 'frames.pdf' in the current directory"
echo ""

# Run for 5 seconds to collect TF data
echo "Collecting TF data for 5 seconds..."
timeout 5 ros2 run tf2_tools view_frames || {
    echo "Warning: tf2_tools may have exited early. Checking for output..."
}

# Wait a moment for file to be written
sleep 1

# Check if frames.pdf was created
if [ -f "frames.pdf" ]; then
    echo ""
    echo "✓ Successfully generated frames.pdf"
    echo ""
    echo "TF Tree Summary:"
    echo "  Expected frames:"
    echo "    - map (from SLAM/localization)"
    echo "    - odom (from ESP32 or EKF)"
    echo "    - base_link (robot base)"
    echo "    - base_footprint"
    echo "    - chassis"
    echo "    - left_wheel, right_wheel"
    echo "    - laser_frame (lidar)"
    echo "    - camera_link (camera)"
    echo ""

    # Also check if frames.gv was created (graphviz file)
    if [ -f "frames.gv" ]; then
        echo "Frame connections from frames.gv:"
        cat frames.gv | grep -E "^\s*\"" | head -20
        echo ""
    fi

    read -p "Press Enter to open frames.pdf or Ctrl+C to skip..."
    $VIEWER frames.pdf &
    echo ""
    echo "Note: You can also manually open 'frames.pdf' with any PDF viewer"
else
    echo ""
    echo "✗ Failed to generate frames.pdf"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Make sure the robot is running: ros2 launch lunohod-1 launch_robot.launch.py"
    echo "  2. Check TF topics: ros2 topic list | grep tf"
    echo "  3. Check TF manually: ros2 run tf2_ros tf2_echo odom base_link"
    echo "  4. Verify robot_state_publisher is running: ros2 node list | grep robot_state_publisher"
    exit 1
fi

echo "=========================================="
echo "Visualization Complete!"
echo "=========================================="
echo ""
echo "To check specific transforms manually:"
echo "  ros2 run tf2_ros tf2_echo <source_frame> <target_frame>"
echo ""
echo "Examples:"
echo "  ros2 run tf2_ros tf2_echo odom base_link"
echo "  ros2 run tf2_ros tf2_echo base_link laser_frame"
echo "  ros2 run tf2_ros tf2_echo map odom"
echo ""
