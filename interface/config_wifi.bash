#!/bin/bash

# WiFi SSID and password
SSID="your_wifi_ssid"
PASSWORD="your_wifi_password"

# Network interface (e.g., wlan0)
INTERFACE="wlan0"

# Connect to the WiFi network
sudo nmcli device wifi connect "$SSID" password "$PASSWORD" ifname "$INTERFACE"

# Display connection status
sudo nmcli connection show

echo "WiFi configuration completed."
