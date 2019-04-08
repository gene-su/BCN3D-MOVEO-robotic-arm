#!/bin/bash

# Getting offsets from camera which connects to Raspberry Pi3

while :
do 
	sleep 0.5
	sshpass -p 417 scp pi@192.168.43.185:/home/pi/offset/offset.npy ./
	echo "OK"
done
