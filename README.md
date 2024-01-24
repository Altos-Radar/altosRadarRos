# altosRadarRos
1. git clone git@github.com:Altos-Radar/altosRadarRos.git
2. cd altosRadarRos
3. catkin_make
4. open terminal in altosRadarRos
   roscore
5. open terminal in altosRadarRos
   rosrun rviz rviz
   open File-->Open Config-->choose altosRadarRos/src/altosradar/altosradar.rviz
6. open terminal in altosRadarRos
   source devel/setup.bash
   rosrun  altosradar altosRadarParse  
   
------------------------------------------------------------------------------------   
   
   
OR
1. git clone git@github.com:Altos-Radar/altosRadarRos.git
2. cd altosRadarRos
3. catkin_make
4. open terminal in altosRadarRos
   ./start.sh
   
  rosbag replay: 
  1. catkin_make
  2. source devel/setup.bash
  3. roslaunch baidu_demo demo.launch
  4. rosbag play xxx.bag
