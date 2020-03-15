rosnode kill rangefinder
echo "Run laser sensor vl53l1x"
/usr/bin/screen -Dm bash -c " source /home/pi/catkin_ws/devel/setup.bash && roslaunch clever_telegram range_vl53l1x.launch" &

