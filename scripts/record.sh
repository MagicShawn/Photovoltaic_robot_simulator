

cd /home/maxliang/Project/Mine/BiShe/Reference/Photovoltaic_robot_simulator/rosbag
rosbag record -O ${BAG_NAME} /odom /navsat/fix /navsat/vel /gazebo/model_states /imu/data /cmd_vel /move_base/PathPlanner/plan


cd /home/maxliang/Project/Mine/BiShe/Reference/Photovoltaic_robot_simulator/data
rostopic echo /move_base/PathPlanner/plan > ${FILE_NAME}.txt