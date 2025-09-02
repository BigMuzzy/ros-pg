# DevContainer Configurations

This repository supports multiple DevContainer configurations for different architectures:

## Available Configurations

### 1. x86_64 (Default) - `.devcontainer/`
- **Platform**: Linux x86_64 
- **Base Image**: `althack/ros2:kilted-dev`
- **Use Case**: Development machines, CI/CD, most laptops/desktops
- **Command**: Open folder in VS Code (default configuration)

### 2. ARM64/RPi5 - `.devcontainer-arm64/`
- **Platform**: Linux ARM64 
- **Base Image**: `arm64v8/ros:kilted-perception`
- **Use Case**: Raspberry Pi 4/5, ARM64 development boards
- **Command**: Select "Dev Containers: Reopen in Container" and choose the ARM64 configuration

## Usage Instructions

### For x86_64 Development (Default)
1. Open the project in VS Code
2. When prompted, click "Reopen in Container" 
3. VS Code will use `.devcontainer/devcontainer.json` automatically

### For ARM64/Raspberry Pi Development

#### Option 1: VS Code
1. Open the project in VS Code
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
3. Type "Dev Containers: Reopen in Container"
4. Select "From 'devcontainer.json'..."
5. Choose the `.devcontainer-arm64/devcontainer.json` configuration

#### Option 2: DevContainer CLI
1. Start the container:
   ```bash
   sudo devcontainer up --workspace-folder . --config ./.devcontainer-arm64/devcontainer.json
   ```
2. Connect to the running container:
   ```bash
   sudo devcontainer exec --workspace-folder . --config ./.devcontainer-arm64/devcontainer.json /bin/bash
   ```

## Key Differences

| Feature | x86_64 | ARM64/RPi5 |
|---------|--------|------------|
| Base Image | `althack/ros2:kilted-dev` | `arm64v8/ros:kilted-perception` |
| Platform | `linux/amd64` | `linux/arm64` |
| User Setup | Pre-configured | Created during build |
| Performance | Native | Native on ARM64 hardware |

## Hardware Requirements

### x86_64
- Intel/AMD processors
- 4GB+ RAM recommended
- Works with Docker Desktop on Windows/Mac

### ARM64/RPi5
- Raspberry Pi 4/5 (recommended: Pi 5)
- 4GB+ RAM (8GB recommended for Pi 5)
- ARM64 Linux host OS
- Docker installed on the host

## Troubleshooting

### ARM64 Issues
- Ensure you're running 64-bit OS on Raspberry Pi
- Install Docker on the Pi host system
- For serial device access, the container runs in privileged mode
- Some packages may need compilation from source on ARM64

### Performance Notes
- ARM64: Native performance on ARM hardware
- x86_64: Can run on ARM via emulation but will be slower

## Serial Device Access
Both configurations include:
- `/dev` volume mapping for device access
- `dialout` group membership for serial ports
- Device cgroup rules for USB devices