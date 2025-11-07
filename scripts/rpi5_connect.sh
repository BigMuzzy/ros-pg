#!/bin/bash
# RPI5 Script to Connect to ROS2-Robot Hotspot
# Run this on your Raspberry Pi 5

set -e

# Configuration
HOTSPOT_SSID="ROS2-Robot"
HOTSPOT_PASSWORD="robotros2024"
LAPTOP_IP="10.42.0.1"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== RPI5 ROS2 Network Connection ===${NC}"
echo ""

# Check if running with sudo
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run with sudo${NC}"
    exit 1
fi

# Check if NetworkManager is available
if command -v nmcli &> /dev/null; then
    echo -e "${YELLOW}Using NetworkManager to connect...${NC}"

    # Delete existing connection if it exists
    if nmcli connection show "$HOTSPOT_SSID" &> /dev/null; then
        nmcli connection delete "$HOTSPOT_SSID"
    fi

    # Connect to the hotspot
    nmcli device wifi connect "$HOTSPOT_SSID" password "$HOTSPOT_PASSWORD"

    echo -e "${GREEN}✓ Connected to $HOTSPOT_SSID${NC}"

else
    echo -e "${YELLOW}NetworkManager not found, using wpa_supplicant...${NC}"

    # Find wireless interface
    WIFI_INTERFACE=$(ip link | grep -E "^[0-9]+: wl" | awk '{print $2}' | sed 's/://' | head -n1)

    if [ -z "$WIFI_INTERFACE" ]; then
        echo -e "${RED}No wireless interface found${NC}"
        exit 1
    fi

    echo -e "Using interface: ${YELLOW}$WIFI_INTERFACE${NC}"

    # Create temporary wpa_supplicant config
    WPA_CONF="/tmp/wpa_ros2.conf"
    cat > "$WPA_CONF" << EOF
network={
    ssid="$HOTSPOT_SSID"
    psk="$HOTSPOT_PASSWORD"
}
EOF

    # Kill existing wpa_supplicant
    killall wpa_supplicant 2>/dev/null || true
    sleep 1

    # Start wpa_supplicant
    wpa_supplicant -B -i "$WIFI_INTERFACE" -c "$WPA_CONF"
    sleep 2

    # Get IP via DHCP
    dhclient -r "$WIFI_INTERFACE" 2>/dev/null || true
    dhclient "$WIFI_INTERFACE"

    echo -e "${GREEN}✓ Connected to $HOTSPOT_SSID${NC}"
fi

echo ""
echo -e "${YELLOW}Checking network status...${NC}"

# Show IP address
IP_ADDR=$(ip -4 addr show | grep -oP '(?<=inet\s)10\.42\.0\.\d+' | head -n1)
if [ -n "$IP_ADDR" ]; then
    echo -e "  RPI5 IP: ${GREEN}$IP_ADDR${NC}"
else
    echo -e "  ${RED}Warning: No IP in 10.42.0.x range found${NC}"
    echo -e "  Current IPs:"
    ip -4 addr show | grep inet | awk '{print "    " $2}'
fi

# Test connectivity to laptop
echo ""
echo -e "${YELLOW}Testing connectivity to laptop ($LAPTOP_IP)...${NC}"
if ping -c 2 -W 2 "$LAPTOP_IP" &> /dev/null; then
    echo -e "  ${GREEN}✓ Can reach laptop${NC}"
else
    echo -e "  ${RED}✗ Cannot reach laptop${NC}"
fi

# Test internet connectivity
echo -e "${YELLOW}Testing internet connectivity...${NC}"
if ping -c 2 -W 2 8.8.8.8 &> /dev/null; then
    echo -e "  ${GREEN}✓ Internet is working${NC}"
else
    echo -e "  ${RED}✗ No internet access${NC}"
fi

echo ""
echo -e "${GREEN}=== Next Steps ===${NC}"
echo "1. Verify ROS2 environment is sourced:"
echo "   source /opt/ros/humble/setup.bash  # or your ROS2 distro"
echo ""
echo "2. Test ROS2 communication:"
echo "   ros2 topic list"
echo "   ros2 topic echo /rosout"
echo ""
echo "3. Set ROS_DOMAIN_ID (should match laptop):"
echo "   export ROS_DOMAIN_ID=42"
echo ""

# Check if ROS2 is installed
if [ -d "/opt/ros" ]; then
    echo -e "${YELLOW}ROS2 installations found:${NC}"
    ls -d /opt/ros/* 2>/dev/null | xargs -n1 basename | sed 's/^/  - /'
fi

echo ""
