roslaunch robot_description gazebo.launch
roslaunch robot_description controller.launch

# move control
roslaunch robot_control velocity_controller.launch 
# start to control robot by Keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py