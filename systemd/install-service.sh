#!/bin/sh
# Installs armbot.service so the dashboard server starts at boot. It does not start it now;
# stop any server running in a terminal first, then: sudo systemctl start armbot
set -e
sudo cp "$(dirname "$0")/armbot.service" /etc/systemd/system/armbot.service
sudo systemctl daemon-reload
sudo systemctl enable armbot
echo "Installed. Start now with: sudo systemctl start armbot   (log: journalctl -u armbot -f)"
