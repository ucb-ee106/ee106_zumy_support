#! /bin/bash

result="0"
counter=0
count_time=3
host="m92p-3.local"

while [ "$result" != "1" ]; do

  result=$(nmap "$host" -sP | grep -o -P '..host up'| awk '{print $1}')
  #echo "$result"
  counter=$((counter+1))
  echo "$counter"
  
  if [ "$counter" = "$count_time" ]; then
    echo "\nGiving up and rebooting...\n"
    python home/zumy/zumy_workspace/src/zumy_ros/src/ping_light.py
    exit 0
    # echo odroid | sudo -S reboot
  fi
  if [ "$result" = "1" ]; then
     echo "\nWireless network found\n"
  else
     printf "\nNo wireless connection\nTrying to reconnect...\n\n"
     echo odroid | sudo -S ifdown wlan0
     echo odroid | sudo -S ifup wlan0
  fi



done

python home/zumy/zumy_workspace/src/zumy_ros/src/ping_light.py