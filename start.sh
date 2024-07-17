source devel/setup.bash
gnome-terminal --tab -t "Camera" -e  'bash -c "roslaunch usb_cam usb_cam.launch;read"'
sleep 2s
gnome-terminal --tab -t "rviz" -e  'bash -c "roslaunch rviz.launch;read"'
sleep 3s
gnome-terminal --tab -t "altosradar" -e  'bash -c "rosrun altosradar altosRadarParse"'
gnome-terminal --tab -t "ARS548" -e 'bash -c "rosrun ars548 ars548DataParse;read;read"' 
gnome-terminal --tab -t "Lidar" -e 'bash -c "roslaunch pandar_pointcloud PandarSwift_points.launch; read"'
