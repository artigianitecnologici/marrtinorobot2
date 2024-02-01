#!/bin/bash

WLAN0="wlan0"
WLAN1="wlan1"
BRIDGE="br0"
BRIDGE_IP="192.168.1.1"
GATEWAY="192.168.1.254"

# Funzione per verificare se un comando ha successo
check_command() {
    if [ $? -ne 0 ]; then
        echo "Errore: Il comando $1 ha restituito un codice di uscita non zero."
        exit 1
    fi
}

# Verifica l'esistenza delle interfacce WiFi
if ! nmcli device show $WLAN0 > /dev/null 2>&1; then
    echo "Errore: L'interfaccia $WLAN0 non esiste."
    exit 1
fi

if ! nmcli device show $WLAN1 > /dev/null 2>&1; then
    echo "Errore: L'interfaccia $WLAN1 non esiste."
    exit 1
fi

# Creazione del bridge e aggiunta delle interfacce WiFi
sudo nmcli con add type bridge ifname $BRIDGE
check_command "Creazione del bridge"

sudo nmcli con add type bridge-slave ifname $WLAN0 master $BRIDGE
check_command "Aggiunta $WLAN0 al bridge"

sudo nmcli con add type bridge-slave ifname $WLAN1 master $BRIDGE
check_command "Aggiunta $WLAN1 al bridge"

# Configurazione degli indirizzi IP del bridge
sudo nmcli con modify $BRIDGE ipv4.addresses "$BRIDGE_IP/24"
check_command "Configurazione indirizzo IP del bridge"

sudo nmcli con modify $BRIDGE ipv4.gateway "$GATEWAY"
check_command "Configurazione del gateway del bridge"

sudo nmcli con modify $BRIDGE ipv4.method manual
check_command "Impostazione del metodo IP manuale"

# Abilitazione del forwarding IP in modo persistente
echo "net.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.conf > /dev/null
check_command "Abilitazione del forwarding IP in modo persistente"
sudo sysctl -p
check_command "Applicazione delle modifiche al sysctl"

# Aggiunta delle regole di routing
sudo iptables -t nat -A POSTROUTING -o $BRIDGE -j MASQUERADE
check_command "Aggiunta delle regole di routing NAT"

# Riavvio di NetworkManager
sudo systemctl restart NetworkManager
check_command "Riavvio di NetworkManager"

echo "Configurazione completata."
