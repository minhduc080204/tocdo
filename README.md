catkin_make

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/minhduc/catkin_ws/src/turtlebot3_custom_speed_signs/models

source ~/.bashrc

roslaunch turtlebot3_custom_speed_signs my_speed_world_with_camera.launch


rosrun speed_sign_control speed_sign_control.py

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

rqt_image_view