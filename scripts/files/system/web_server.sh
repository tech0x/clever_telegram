rosnode kill web_video_server
echo "Run web_server"
/usr/bin/screen -Dm bash -c " source /home/pi/catkin_ws/devel/setup.bash && roslaunch clever_telegram web_server.launch" &

