# altosRadarRos
1. git clone git@github.com:Altos-Radar/altosRadarRos.git
2. cd altosRadarRos
3. catkin_make
4. Start
   a. open terminal in altosRadarRos
       roscore
   b. open terminal in altosRadarRos
       rosrun rviz rviz
       open File-->Open Config-->choose altosradar.rviz
   c. open terminal in altosRadarRos
       source devel/setup.bash
       rosrun  altosradar altosRadarParse
