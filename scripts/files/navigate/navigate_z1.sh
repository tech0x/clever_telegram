#!/bin/sh

rosservice call /navigate "{z: 1.0, frame_id: 'body', auto_arm: true}"
