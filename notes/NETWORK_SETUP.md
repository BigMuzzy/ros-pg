# ROS2 Robot Network Setup Guide

This guide helps you set up network communication between your laptop and RPI5 robot while maintaining internet access.

## Network Architecture

```
Internet
   ↓
[Built-in WiFi (wlp2s0)] ← Laptop → [AWUS1900 AP (enx806d971f1582)]
                                              ↓
                                         [RPI5 Robot]
```

- **Built-in WiFi**: Connects to internet (e.g., "BlackHole" network)
- **AWUS1900**: Creates WiFi Access Point for robot
- **IP Forwarding**: Shares internet from built-in WiFi to AWUS1900 AP

## Quick Setup

### On Laptop (Linux)

1. **Connect built-in WiFi to internet** (if not already connected):
   ```bash
   nmcli device wifi list
   nmcli device wifi connect "YourNetworkName" password "YourPassword"
   ```

2. **Run the setup script**:
   ```bash
   cd /home/max/projects/ros-pg
   sudo ./setup_robot_network.sh
   ```

3. **Note the network details**:
   - SSID: `ROS2-Robot`
   - Password: `robotros2024` (change in script if desired)
   - Laptop IP on hotspot: `10.42.0.1`

### On RPI5 Robot

#### Option 1: Using nmcli (if NetworkManager is installed)

```bash
# Connect to the hotspot
sudo nmcli device wifi connect "ROS2-Robot" password "robotros2024"

# Verify connection
ip addr show
ping 10.42.0.1  # Ping laptop
ping 8.8.8.8    # Test internet access
```

#### Option 2: Using wpa_supplicant (manual configuration)

1. Edit `/etc/wpa_supplicant/wpa_supplicant.conf`:
   ```bash
   sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
   ```

2. Add this network configuration:
   ```
   network={
       ssid="ROS2-Robot"
       psk="robotros2024"
       priority=10
   }
   ```

3. Restart networking:
   ```bash
   sudo systemctl restart dhcpcd
   # or
   sudo systemctl restart networking
   ```

## ROS2 Configuration

### Setting ROS_DOMAIN_ID (Recommended)

On both laptop and RPI5, set the same domain ID to isolate your ROS2 network:

```bash
# Add to ~/.bashrc on both machines
export ROS_DOMAIN_ID=42  # Use any number 0-101
```

### Configure ROS2 DDS for WiFi

On both laptop and RPI5, create a DDS config file for better WiFi performance:

**File: `/home/max/fastdds_wifi.xml`** (and same on RPI5)

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>wifi_transport</transport_id>
            <type>UDPv4</type>
            <sendBufferSize>1048576</sendBufferSize>
            <receiveBufferSize>4194304</receiveBufferSize>
            <TTL>1</TTL>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="wifi_participant" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>wifi_transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
```

Then set environment variable:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/max/fastdds_wifi.xml
```

## Verification Steps

### 1. Check Laptop Network Status

```bash
# See all connections
nmcli connection show

# Check hotspot status
nmcli connection show ROS2-Robot

# See connected devices
ip neigh show dev enx806d971f1582

# Check IP forwarding is enabled
sysctl net.ipv4.ip_forward
```

### 2. Check RPI5 Network Status

```bash
# Check IP address (should be 10.42.0.x)
ip addr show

# Check route to internet
ip route

# Test connectivity
ping 10.42.0.1      # Laptop
ping 8.8.8.8        # Internet
```

### 3. Test ROS2 Communication

**On Laptop:**
```bash
source /opt/ros/<your-ros-distro>/setup.bash
ros2 topic list
ros2 topic pub /test std_msgs/String "data: Hello from laptop"
```

**On RPI5:**
```bash
source /opt/ros/<your-ros-distro>/setup.bash
ros2 topic list  # Should see /test topic
ros2 topic echo /test
```

## Managing the Hotspot

### Start the hotspot:
```bash
sudo nmcli connection up ROS2-Robot
```

### Stop the hotspot:
```bash
sudo nmcli connection down ROS2-Robot
```

### Auto-start on boot (optional):
```bash
sudo nmcli connection modify ROS2-Robot autoconnect yes
```

### View hotspot details:
```bash
nmcli connection show ROS2-Robot
```

### Change hotspot password:
```bash
sudo nmcli connection modify ROS2-Robot wifi-sec.psk "NewPassword123"
sudo nmcli connection down ROS2-Robot
sudo nmcli connection up ROS2-Robot
```

## Troubleshooting

### AWUS1900 Not Working as AP

If the AWUS1900 shows as ethernet instead of WiFi:

1. **Check driver support:**
   ```bash
   lsusb | grep -i realtek
   iw list  # Should show AP mode support
   ```

2. **Install/update RTL8814AU driver:**
   ```bash
   # Check if driver is loaded
   lsmod | grep 8814

   # If not, you may need to install the driver
   # See: https://github.com/morrownr/8814au
   ```

3. **Try different USB port:**
   - USB 3.0 ports sometimes have issues
   - Try USB 2.0 port

### RPI5 Can't See the Hotspot

1. **Check WiFi band:**
   - Script uses 2.4GHz (wifi.band bg)
   - RPI5 should support this

2. **Scan for networks on RPI5:**
   ```bash
   sudo nmcli device wifi list
   # or
   sudo iwlist wlan0 scan | grep ROS2-Robot
   ```

### No Internet on RPI5

1. **Check IP forwarding on laptop:**
   ```bash
   sudo sysctl -w net.ipv4.ip_forward=1
   # Make permanent:
   echo "net.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.conf
   ```

2. **Check iptables rules:**
   ```bash
   sudo iptables -t nat -L -n -v
   # Should see MASQUERADE rule
   ```

3. **Test from RPI5:**
   ```bash
   ping 10.42.0.1      # Can reach laptop?
   ping 8.8.8.8        # Can reach internet?
   traceroute 8.8.8.8  # Where does it fail?
   ```

### ROS2 Topics Not Visible

1. **Check ROS_DOMAIN_ID matches:**
   ```bash
   # On both machines:
   echo $ROS_DOMAIN_ID
   ```

2. **Check multicast is working:**
   ```bash
   # On laptop:
   ros2 multicast receive

   # On RPI5 (in another terminal):
   ros2 multicast send
   ```

3. **Disable firewall temporarily for testing:**
   ```bash
   # On laptop:
   sudo ufw disable  # Remember to re-enable later!
   ```

4. **Check network interfaces in ROS2:**
   ```bash
   # On both machines, verify correct interface is used
   ip route get 8.8.8.8  # Should use correct interface
   ```

## Network Information

| Device | Interface | IP Address | Role |
|--------|-----------|------------|------|
| Laptop | wlp2s0 | DHCP from router | Internet access |
| Laptop | enx806d971f1582 | 10.42.0.1 | WiFi AP for robot |
| RPI5 | wlan0 | 10.42.0.x (DHCP) | Robot client |

## Security Notes

- Change the default password `robotros2024` to something secure
- The hotspot uses WPA2-PSK encryption
- Consider using a stronger password (12+ characters)
- IP forwarding exposes your robot to the internet - ensure firewall rules are appropriate

## Performance Tips

1. **Keep devices close:** WiFi range affects latency
2. **Use 5GHz if both support it:** Modify script to use `wifi.band a` for 5GHz
3. **Monitor bandwidth:** Use `iftop` or `nload` to check network usage
4. **Reduce ROS2 QoS:** Use BEST_EFFORT instead of RELIABLE for sensor data where appropriate

## References

- [ROS2 DDS Tuning](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html)
- [NetworkManager WiFi Hotspot](https://docs.fedoraproject.org/en-US/quick-docs/creating-a-wifi-hotspot/)
- [RTL8814AU Driver](https://github.com/morrownr/8814au)
