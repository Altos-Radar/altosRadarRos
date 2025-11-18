#!/bin/bash
# @Author: Fengze Han
# @Date:   2025-01-17 15:17:44
# @Last Modified by:   Fengze Han
# @Last Modified time: 2025-11-12 15:29:52
source devel/setup.bash
gnome-terminal --tab -t "rviz" -e  'bash -c "roslaunch rviz.launch;read"'
sleep 1s
gnome-terminal --tab -t "altosradar" -e  'bash -c "rosrun altosradar altosRadarParse 4040;read"'



