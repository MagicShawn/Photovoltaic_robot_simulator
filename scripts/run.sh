roslaunch robot_description gazebo.launch
roslaunch robot_description controller.launch

roslaunch fast

# move control
roslaunch robot_control velocity_controller.launch 
# start to control robot by Keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# 启动算法仿真RViz
rosrun rviz rviz -d /home/graduation/Work_space/Photovoltaic_robot_simulator/src/slam_algorithm/FAST_LIO/rviz_cfg/loam_livox.rviz
# 在独立命名空间中启动Gazebo仿真RViz
ROS_NAMESPACE=gazebo_rviz rosrun rviz rviz -d /home/graduation/Work_space/Photovoltaic_robot_simulator/src/robot_simulator/robot_description/launch/urdf.rviz