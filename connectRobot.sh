sudo ip addr flush dev enP8p1s0 #enp0s31f6
sudo ip addr add 192.168.5.100/24 dev enP8p1s0 #enp0s31f6
sudo ip link set dev enP8p1s0 up #enp0s31f6 up
ssh ysc@192.168.5.1 # po Ethernet

#ssh ysc@192.168.2.1 # po WiFi
#pass = "'"