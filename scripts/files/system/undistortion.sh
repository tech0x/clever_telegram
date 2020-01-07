#!/bin/bash

pid=`ps ax | grep python | grep undistort_camera.py | grep -v grep | grep -v screen | grep -v bash |awk '{print $1}'`
echo $pid
if [ $pid -gt "0" ];
then
  echo "Kill previous undistortion node"
  kill -9 $pid
fi

dist=`rostopic echo -n 1  /main_camera/camera_info | grep 'D: \[\]' | wc -l`
if [ $dist -eq "1" ];
then
  echo "Distortion model is empty, check camera config file"
  exit
fi

pid=`ps ax | grep python | grep undistort_camera.py | grep -v grep | grep -v screen | grep -v bash |awk '{print $1}'`
if [ !$pid ];
then
  echo "Run undistortion node"
  /usr/bin/screen -Dm bash -c "python /opt/ros/kinetic/lib/clever_telegram/scripts/undistort_camera.py " &
fi

