echo "Run web_server"
roslaunch clever_telegram undistortion.launch
/usr/bin/screen -Dm bash -c " source /home/pi/catkin_ws/devel/setup.bash && roslaunch clever_telegram web_server.launch" &

