echo "Run cv_camera"
rosnode kill main_camera
/usr/bin/screen -Dm bash -c " source /home/pi/catkin_ws/devel/setup.bash && roslaunch clever main_camera.launch" &


