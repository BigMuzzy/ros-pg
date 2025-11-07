#!/bin/bash
# Setup WiFi Access Point on AWUS1900 for ROS2 Robot Communication
# This script creates a hotspot that shares internet from built-in WiFi

set -e

# Configuration
HOTSPOT_NAME="ROS2-Robot"
HOTSPOT_PASSWORD="robotros2024"  # Change this to your preferred password (min 8 chars)
HOTSPOT_INTERFACE="wlx00c0caae8c66"  # AWUS1900 adapter
INTERNET_INTERFACE="wlp2s0"  # Built-in WiFi

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ROS2 Robot Network Setup ===${NC}"
echo ""

# Check if running with sudo
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run with sudo${NC}"
    exit 1
fi

# Check if NetworkManager is installed
if ! command -v nmcli &> /dev/null; then
    echo -e "${RED}NetworkManager (nmcli) is not installed${NC}"
    exit 1
fi

# Check if the hotspot interface exists
if ! ip link show "$HOTSPOT_INTERFACE" &> /dev/null; then
    echo -e "${YELLOW}Warning: Interface $HOTSPOT_INTERFACE not found${NC}"
    echo "Available interfaces:"
    ip link show | grep -E "^[0-9]+:" | awk '{print "  - " $2}' | sed 's/://'
    exit 1
fi

echo -e "${YELLOW}Step 1: Checking current connections...${NC}"
nmcli connection show

# Delete existing hotspot connection if it exists
if nmcli connection show "$HOTSPOT_NAME" &> /dev/null; then
    echo -e "${YELLOW}Removing existing hotspot connection...${NC}"
    nmcli connection delete "$HOTSPOT_NAME"
fi

echo -e "${YELLOW}Step 2: Creating WiFi hotspot on $HOTSPOT_INTERFACE...${NC}"

# Create the hotspot connection with internet sharing
nmcli connection add type wifi ifname "$HOTSPOT_INTERFACE" \
    con-name "$HOTSPOT_NAME" \
    autoconnect no \
    ssid "$HOTSPOT_NAME" \
    wifi.mode ap \
    wifi.band bg \
    ipv4.method shared \
    ipv6.method ignore

# Set the password
nmcli connection modify "$HOTSPOT_NAME" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$HOTSPOT_PASSWORD"

echo -e "${GREEN}✓ Hotspot connection created${NC}"

echo -e "${YELLOW}Step 3: Starting the hotspot...${NC}"

# Bring up the hotspot
nmcli connection up "$HOTSPOT_NAME"

echo -e "${GREEN}✓ Hotspot is now active!${NC}"
echo ""
echo -e "${GREEN}=== Configuration Details ===${NC}"
echo -e "  Network Name (SSID): ${YELLOW}$HOTSPOT_NAME${NC}"
echo -e "  Password:            ${YELLOW}$HOTSPOT_PASSWORD${NC}"
echo -e "  Hotspot Interface:   ${YELLOW}$HOTSPOT_INTERFACE${NC}"
echo -e "  Internet Interface:  ${YELLOW}$INTERNET_INTERFACE${NC}"
echo ""
echo -e "${GREEN}=== Next Steps ===${NC}"
echo "1. On your RPI5, connect to WiFi network: $HOTSPOT_NAME"
echo "2. Use password: $HOTSPOT_PASSWORD"
echo "3. The RPI5 will get an IP in the 10.42.0.x range"
echo "4. Your laptop will have IP: 10.42.0.1 on the hotspot"
echo ""
echo -e "${YELLOW}To stop the hotspot:${NC}"
echo "  sudo nmcli connection down $HOTSPOT_NAME"
echo ""
echo -e "${YELLOW}To start the hotspot again:${NC}"
echo "  sudo nmcli connection up $HOTSPOT_NAME"
echo ""
echo -e "${YELLOW}To check connected devices:${NC}"
echo "  ip neigh show dev $HOTSPOT_INTERFACE"
echo ""
