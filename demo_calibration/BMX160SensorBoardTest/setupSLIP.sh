
sudo slattach -Lvd -p slip -s 115200 /dev/ttyUSB0 &

sleep 1

sudo ifconfig sl0 up pointtopoint 192.168.203.2 192.168.203.1
sleep 1
sudo ifconfig sl0 up pointtopoint 192.168.203.2 192.168.203.1
