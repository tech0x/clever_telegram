echo "Run screen session \"$1\" "
/usr/bin/screen -Dm bash -c " source /home/pi/catkin_ws/devel/setup.bash && $1 " &
