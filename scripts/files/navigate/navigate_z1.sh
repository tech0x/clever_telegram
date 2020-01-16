#!/bin/sh

rosservice call /navigate "{x: 0.0, y: 0.0, z: 1.0, yaw: 1.0, yaw_rate: 0.0, speed: 0.1, frame_id: 'body', auto_arm: true}"
sleep 5
rosservice call /land "{}"
