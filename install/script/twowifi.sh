#!/bin/bash

# Configurazione delle interfacce WiFi e del bridge
WLAN_IN="wlp2s0"
WLAN_OUT="wlx00c0ca985540"
BRIDGE="br0"
BRIDGE_IP="192.168.1.200"
GATEWAY="192.168.1.1"

# Creazione del bridge e aggiunta delle interfacce WiFi
sudo nmcli con add type bridge ifname $BRIDGE
sudo nmcli con add type bridge-slave ifname $WLAN_IN master $BRIDGE
sudo nmcli con add type bridge-slave ifname $WLAN_OUT master $BRIDGE

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
