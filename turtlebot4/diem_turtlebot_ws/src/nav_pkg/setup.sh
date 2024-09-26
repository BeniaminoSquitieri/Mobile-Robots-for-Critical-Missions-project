#!/bin/bash

# Chiedi all'utente di inserire l'indirizzo IP
read -p "IP ADDRESS: " IP_ADDRESS

# Esegui i comandi con l'indirizzo IP fornito dall'utente
sudo ntpdate $IP_ADDRESS
#wget -qO - https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/turtlebot4_discovery/configure_discovery.sh | bash <(cat) </dev/tty
bash /home/francescavenditti/turtlebot4/scripts/configure_discovery.sh
source ~/.bashrc
ros2 daemon stop
ros2 daemon start
sleep 1
ros2 topic list

