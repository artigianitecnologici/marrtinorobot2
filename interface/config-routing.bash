#!/bin/bash

# passaggio parametri 
echo "Script is running."
echo "wlan0 : $1"
echo "wlan1 : $2"
echo "bridge : $3"
echo "bridge_ip : $4"
echo "gateway : $5"
echo "sample:config-routing.bash  wlan0 wlan1 br0 192.168.1.1 192.168.1.254"
# Configurazione delle interfacce WiFi e del bridge
WLAN0=$1
WLAN1=$2
BRIDGE=$3
BRIDGE_IP=$4
GATEWAY=$5

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
