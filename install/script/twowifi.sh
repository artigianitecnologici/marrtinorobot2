#!/bin/bash

# Configurazione delle interfacce WiFi e del bridge
WLAN0="wlan0"
WLAN1="wlan1"
BRIDGE="br0"
BRIDGE_IP="192.168.1.1"
GATEWAY="192.168.1.254"

# Creazione del bridge e aggiunta delle interfacce WiFi
sudo nmcli con add type bridge ifname $BRIDGE
sudo nmcli con add type bridge-slave ifname $WLAN0 master $BRIDGE
sudo nmcli con add type bridge-slave ifname $WLAN1 master $BRIDGE

# Configurazione degli indirizzi IP del bridge
sudo nmcli con modify $BRIDGE ipv4.addresses "$BRIDGE_IP/24"
sudo nmcli con modify $BRIDGE ipv4.gateway "$GATEWAY"
sudo nmcli con modify $BRIDGE ipv4.method manual

# Abilitazione del forwarding IP
sudo sysctl -w net.ipv4.ip_forward=1

# Aggiunta delle regole di routing
sudo iptables -t nat -A POSTROUTING -o $BRIDGE -j MASQUERADE

# Riavvio di NetworkManager
sudo systemctl restart NetworkManager

echo "Configurazione completata."
