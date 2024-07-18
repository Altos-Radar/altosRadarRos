source devel/setup.bash
gnome-terminal --tab -t "rviz" -e  'bash -c "roslaunch rviz.launch;read"'
sleep 3s
gnome-terminal --tab -t "altosradar" -e  'bash -c "rosrun altosradar altosRadarParse;read"'

