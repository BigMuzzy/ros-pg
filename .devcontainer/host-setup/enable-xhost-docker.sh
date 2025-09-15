#!/bin/bash

# Script to enable xhost-docker service for GUI applications in Docker containers

set -e

SERVICE_FILE="xhost-docker.service"
SERVICE_PATH="/etc/systemd/user/$SERVICE_FILE"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Setting up xhost-docker service..."

# Copy service file to systemd user directory
echo "Copying service file to $SERVICE_PATH"
sudo mkdir -p /etc/systemd/user
sudo cp "$SCRIPT_DIR/$SERVICE_FILE" "$SERVICE_PATH"

# Reload systemd and enable the service
echo "Reloading systemd daemon..."
systemctl --user daemon-reload

echo "Enabling xhost-docker service..."
systemctl --user enable xhost-docker.service

echo "Starting xhost-docker service..."
systemctl --user start xhost-docker.service

echo "Checking service status..."
systemctl --user status xhost-docker.service --no-pager

echo ""
echo "✅ xhost-docker service has been enabled and started!"
echo "Docker containers can now access X11 display for GUI applications."
echo ""
echo "To disable: systemctl --user disable xhost-docker.service"
echo "To stop: systemctl --user stop xhost-docker.service"