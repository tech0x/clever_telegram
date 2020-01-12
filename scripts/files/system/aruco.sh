echo "Aruco Mode"
rosnode kill aruco_detect
rosnode kill aruco_map
rosnode kill optical_flow
rosnode kill rangefinder
/usr/bin/screen -Dm bash -c " source /home/pi/catkin_ws/devel/setup.bash && roslaunch clever aruco.launch" &


