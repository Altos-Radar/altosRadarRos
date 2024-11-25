source devel/setup.bash
gnome-terminal --tab -t "rviz" -e  'bash -c "roslaunch rviz.launch;read"'
sleep 3s
gnome-terminal --tab -t "usbcam" -e  'bash -c "roslaunch usb_cam usb_cam.launch;read"'
sleep 1s
gnome-terminal --tab -t "altosradar" -e  'bash -c "rosrun altosradar altosRadarParse;read"'
sleep 1s
gnome-terminal --tab -t "Lidar" -e 'bash -c "roslaunch pandar_pointcloud PandarSwift_points.launch; read"'


