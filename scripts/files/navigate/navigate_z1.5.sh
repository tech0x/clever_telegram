#!/bin/sh

rosservice call /navigate "{z: 1.5, frame_id: 'body', auto_arm: true}"
sleep 5
rosservice call /land "{}"
