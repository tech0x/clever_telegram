#!/bin/sh

rosservice call /navigate "{z: 1.5, frame_id: 'body', auto_arm: true}"
