 #!/bin/bash         

echo "Making both Mbed and Zumy Files"
echo "You'd better run this after both work independantly"

echo "zumy_mbed"
cd zumy_mbed
make
cd ../

echo "zumy_ros"
cd on_zumy_odroid/zumy_ros
rm -rf /build/
rm -rf /devel/
catkin_make

cd ../../
git log > git_log.txt

date > compiled_on.txt
