# Description: This script is used to configure the network and expose the WSL2 to the network.
# Usage: source network.sh

sudo ip addr flush dev eth0
sudo ip addr add 192.168.178.244/24 dev eth0
ip addr show dev eth0
sudo ip link set eth0 up
sudo ip route add default via 192.168.178.1 dev eth0

ip route show
ping 192.168.178.1
ping google.com