# hardware level
## can
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 500000 #500kbps
sudo ip link set up can0
