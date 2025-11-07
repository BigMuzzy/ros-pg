#!/bin/bash
# Install proper RTL8814AU driver for AWUS1900
# This enables WiFi AP mode functionality

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== AWUS1900 Driver Installation ===${NC}"
echo ""
echo "This will install the proper RTL8814AU WiFi driver"
echo "for your AWUS1900 adapter to enable Access Point mode."
echo ""

if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run with sudo${NC}"
    exit 1
fi

# Install dependencies
echo -e "${YELLOW}Step 1: Installing build dependencies...${NC}"
apt update
apt install -y \
    build-essential \
    dkms \
    git \
    bc \
    linux-headers-$(uname -r)

echo -e "${GREEN}✓ Dependencies installed${NC}"
echo ""

# Clone the driver repository
echo -e "${YELLOW}Step 2: Cloning RTL8814AU driver repository...${NC}"
DRIVER_DIR="/usr/src/8814au-git"

if [ -d "$DRIVER_DIR" ]; then
    echo "Removing old driver directory..."
    rm -rf "$DRIVER_DIR"
fi

# Use the well-maintained driver by morrownr
git clone https://github.com/morrownr/8814au.git "$DRIVER_DIR"
cd "$DRIVER_DIR"

echo -e "${GREEN}✓ Driver source downloaded${NC}"
echo ""

# Install the driver using DKMS
echo -e "${YELLOW}Step 3: Installing driver with DKMS...${NC}"

# Create dkms.conf if not exists
if [ ! -f dkms.conf ]; then
    cat > dkms.conf << 'EOF'
PACKAGE_NAME="8814au"
PACKAGE_VERSION="git"
BUILT_MODULE_NAME[0]="8814au"
DEST_MODULE_LOCATION[0]="/kernel/drivers/net/wireless/"
AUTOINSTALL="yes"
EOF
fi

# Add to DKMS
dkms add -m 8814au -v git || true
dkms build -m 8814au -v git
dkms install -m 8814au -v git

echo -e "${GREEN}✓ Driver installed${NC}"
echo ""

# Blacklist the cdc_ether driver for this device
echo -e "${YELLOW}Step 4: Blacklisting conflicting driver...${NC}"

cat > /etc/modprobe.d/blacklist-8814au.conf << 'EOF'
# Prevent cdc_ether from claiming RTL8814AU devices
# This allows the proper 8814au WiFi driver to work
blacklist cdc_ether for RTL8814AU
EOF

echo -e "${GREEN}✓ Driver configuration complete${NC}"
echo ""

echo -e "${YELLOW}Step 5: Unplug and replug your AWUS1900 adapter${NC}"
echo "Or reboot your system to load the new driver"
echo ""
read -p "Press Enter after replugging the adapter (or Ctrl+C to reboot)..."

# Load the new driver
echo -e "${YELLOW}Loading new driver...${NC}"
modprobe -r cdc_ether 2>/dev/null || true
modprobe 8814au

sleep 3

# Check if driver is loaded
if lsmod | grep -q 8814au; then
    echo -e "${GREEN}✓ Driver loaded successfully!${NC}"
    echo ""

    # Show wireless interfaces
    echo -e "${YELLOW}Wireless interfaces detected:${NC}"
    iw dev 2>/dev/null || echo "Run: sudo apt install iw"

    echo ""
    echo -e "${GREEN}=== Installation Complete ===${NC}"
    echo "Your AWUS1900 should now appear as a WiFi interface (e.g., wlx...)"
    echo "You can now run: ${YELLOW}sudo ./setup_robot_network.sh${NC}"
else
    echo -e "${RED}Driver not loaded. You may need to reboot.${NC}"
    echo ""
    read -p "Reboot now? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        reboot
    fi
fi
