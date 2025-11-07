# RPI5 WiFi Priority Management

## The Problem

When you connect your RPI5 to the "ROS2-Robot" hotspot, NetworkManager saves it as a connection profile. On reboot, NetworkManager will try to connect to any saved WiFi networks it can find, which might be:
- Your home WiFi
- The ROS2-Robot hotspot (if laptop is running it)
- Any other saved networks

## The Solution

Use **`rpi5_connect_priority.sh`** which gives you 4 options:

### Option 1: ROS2-Robot ONLY
```bash
sudo ./rpi5_connect_priority.sh
# Choose: 1
```
- ✅ RPI5 will **always** connect to laptop on boot (if available)
- ✅ Disables auto-connect for all other WiFi networks
- ⚠️ If laptop hotspot is off, RPI5 won't have WiFi

**Use case:** Dedicated robot that only needs laptop connection

### Option 2: ROS2-Robot PRIORITY (Recommended)
```bash
sudo ./rpi5_connect_priority.sh
# Choose: 2
```
- ✅ RPI5 prefers laptop hotspot when available
- ✅ Falls back to home WiFi if laptop is off
- ✅ Best of both worlds

**Use case:** Development robot that needs both laptop and home internet

### Option 3: ROS2-Robot MANUAL
```bash
sudo ./rpi5_connect_priority.sh
# Choose: 3
```
- ✅ RPI5 boots with home WiFi (or other saved networks)
- ✅ Manually connect to laptop when needed
- ⚠️ Requires manual connection each time

**Use case:** Robot that usually uses home WiFi, occasionally needs laptop

### Option 4: Keep Current Settings
```bash
sudo ./rpi5_connect_priority.sh
# Choose: 4
```
- ✅ Just connects now, doesn't change boot behavior
- ⚠️ Boot behavior is unpredictable

**Use case:** Testing, don't want to change settings yet

## Manual Connection Control

### Connect to ROS2-Robot manually:
```bash
sudo nmcli connection up ROS2-Robot
```

### Connect to home WiFi manually:
```bash
sudo nmcli connection up "YourHomeWiFi"
```

### Check what will happen on reboot:
```bash
nmcli -f NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show
```

### Change priority later:
```bash
# Make ROS2-Robot highest priority
sudo nmcli connection modify ROS2-Robot connection.autoconnect-priority 100

# Make home WiFi higher priority than ROS2-Robot
sudo nmcli connection modify "YourHomeWiFi" connection.autoconnect-priority 200
```

## How NetworkManager Chooses WiFi on Boot

NetworkManager connects based on:

1. **Autoconnect enabled** (`connection.autoconnect yes`)
2. **Priority number** (higher number = higher priority)
3. **Signal strength** (if priorities are equal)
4. **Last connected** (as tiebreaker)

Default priority is `0` for all networks, so last-connected usually wins.

## Quick Reference

| Mode | Laptop Hotspot Available | Laptop Hotspot Off |
|------|-------------------------|-------------------|
| **ONLY** | ✅ Connects to laptop | ❌ No WiFi |
| **PRIORITY** | ✅ Connects to laptop | ✅ Connects to home WiFi |
| **MANUAL** | ⚠️ Needs manual connect | ✅ Connects to home WiFi |
| **CURRENT** | 🤷 Depends on settings | 🤷 Depends on settings |

## Recommendations

- **For development:** Use **Option 2** (PRIORITY) - seamless switching
- **For demos:** Use **Option 1** (ONLY) - guaranteed laptop connection
- **For home use:** Use **Option 3** (MANUAL) - usually on home WiFi

## Troubleshooting

### RPI5 connects to wrong network on boot

Check current priorities:
```bash
nmcli -f NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show
```

Set ROS2-Robot to highest:
```bash
sudo nmcli connection modify ROS2-Robot connection.autoconnect-priority 100
```

### Want to temporarily disable a network

```bash
sudo nmcli connection modify "NetworkName" connection.autoconnect no
```

Re-enable:
```bash
sudo nmcli connection modify "NetworkName" connection.autoconnect yes
```

### Check which network RPI5 is connected to

```bash
nmcli connection show --active
```

### Force disconnect and reconnect

```bash
sudo nmcli connection down ROS2-Robot
sudo nmcli connection up ROS2-Robot
```
