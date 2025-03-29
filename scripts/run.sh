# 启动机器人人仿真
roslaunch robot_description gazebo.launch

roslaunch robot_description controller.launch

# 启动fastlio
roslaunch fast_lio mapping_velodyne.launch
# 启动fastlio_sam
roslaunch fast_lio_sam mapping_velodyne16_lio_sam_parking_dataset.launch

# move control
roslaunch robot_control velocity_controller.launch 
# start to control robot by Keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# 启动算法仿真RViz
rosrun rviz rviz -d $(rospack find fast_lio_sam)/rviz_cfg/loam_livox.rviz
rosrun rviz rviz -d $(rospack find fast_lio)/rviz_cfg/loam_livox.rviz
# 在独立命名空间中启动Gazebo仿真RViz
ROS_NAMESPACE=gazebo_rviz rosrun rviz rviz -d $(rospack find robot_description)/launch/urdf.rviz