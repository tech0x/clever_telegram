echo "Run undistortion node"
/usr/bin/screen -Dm bash -c " source /home/pi/catkin_ws/devel/setup.bash && roslaunch clever_telegram undistortion.launch" &

