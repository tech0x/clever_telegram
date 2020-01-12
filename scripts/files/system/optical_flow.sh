echo "Optical Flow Mode"
rosnode kill aruco_detect
rosnode kill optical_flow
rosnode kill rangefinder
/usr/bin/screen -Dm bash -c " source /home/pi/catkin_ws/devel/setup.bash && roslaunch clever_telegram range_vl53l1x.launch" &
/usr/bin/screen -Dm bash -c " source /home/pi/catkin_ws/devel/setup.bash && roslaunch clever_telegram optical_flow.launch" &

