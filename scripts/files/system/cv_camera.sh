echo "Run cv_camera"
rosnode kill main_camera
/usr/bin/screen -Dm bash -c "roslaunch clever main_camera.launch" &


