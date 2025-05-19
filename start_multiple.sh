# run multiple altosRadarParse node at the same time
source devel/setup.bash
# gnome-terminal --tab -t "rviz" -e  'bash -c "roslaunch rviz.launch node_name:=rviz0;read"'
# sleep 1s
# gnome-terminal --tab -t "rviz" -e  'bash -c "roslaunch rviz.launch node_name:=rviz1;read"'
# sleep 1s
# gnome-terminal --tab -t "rviz" -e  'bash -c "roslaunch rviz.launch node_name:=rviz2;read"'
# sleep 1s

# start roscore
gnome-terminal --tab -t "roscore" -e  'bash -c "roscore;read"'
sleep 2s
gnome-terminal --tab -t "altosradar0" -e  'bash -c "rosrun altosradar altosRadarParse 4040 __name:=altos_instance_0;read"'
gnome-terminal --tab -t "altosradar1" -e  'bash -c "rosrun altosradar altosRadarParse 4041 __name:=altos_instance_1;read"'
gnome-terminal --tab -t "altosradar2" -e  'bash -c "rosrun altosradar altosRadarParse 4042 __name:=altos_instance_2;read"'
# etc...

