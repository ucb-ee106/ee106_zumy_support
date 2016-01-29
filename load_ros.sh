 #!/bin/bash         

#need to install 'sshpass'
echo "Loading both Mbed and Zumy Files"
echo "You'd better be ready for the possibility of loosing files"
echo ""

NAME="$1"
echo "Loading to " $NAME

echo "Deleting everything"
ssh "zumy@"$NAME".local" "sudo rm -rf *"

echo "Loading ROS code"
cd on_zumy_odroid/
scp -rp .* "zumy@"$NAME".local:"
cd ../

#echo "Loading Mbed .bin file"
#cd zumy_mbed/ 
#copy the bin file over to a temp location
#scp "zumy_mbed.bin" "zumy@"$NAME".local:"

#ssh in and mount the mbed files.  I'm going to need root for this
#spawn sshpass -p "zumy" ssh -o StrictHostKeyChecking=no "zumy@"$NAME".local"

#ssh -t "zumy@"$NAME".local" "sudo mkdir -p "/media/boot/mbed" ; sudo mount "/dev/sda" "/media/boot/mbed/" ; sudo cp -f "/home/zumy/zumy_mbed.bin" "/media/boot/mbed" ; sync"

#make a file with the date & time, but it into the zumy for informational purposes
dt=`date '+%d/%m/%Y %H:%M:%S'`
echo "$dt" > "last_loaded.txt"
scp "last_loaded.txt" "zumy@"$NAME".local:"

exit


