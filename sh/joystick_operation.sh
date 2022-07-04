#! /bin/bash

source ./params
source ../../../devel/setup.bash

linear_vel_coef=1.0
angular_vel_coef=1.0

rosrun joy joy_node &
roslaunch tello_navi joy_ope_node.launch tello_num:=$tello_num linear_vel_coef:=$linear_vel_coef angular_vel_coef:=$angular_vel_coef &

sleep 3
echo -e "\n\n"
read -p "Hit enter to shutdown the joystick operation system: "

rosnode kill /joy_node
rosnode kill /joy_ope_node
