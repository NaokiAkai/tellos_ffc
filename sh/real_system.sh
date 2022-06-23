#! /bin/bash

source ./params
source ../devel/setup.bash


# Run the motion capture node.
roslaunch vrpn_client_ros tellos_mocap.launch &


# Run TF nodes.
# Assume the positions measured by the motion capture and base link of the tellos are the same.
for ((i = 0; i < $tello_num; i++))
do
node_name="tello${i}_to_base_link${i}"
rosrun tf static_transform_publisher 0 0 0 0 0 0 /tello${i} /base_link${i} 100 __name:=$node_name &
done


# Run the operation node for the tellos using a joy stick.
# This node is used for emergency stop.
rosrun joy joy_node &
rosrun tello_navi joy_ope_node &




sleep 3
echo -e "\n\n"
read -p "Hit enter to shutdown the real experiment system: "



rosnode kill /vrpn_client_node

for ((i = 0; i < $tello_num; i++))
do
tf_node_name="/tello${i}_to_base_link${i}"
rosnode kill $tf_node_name
done

rosnode kill /joy_node
rosnode kill /joy_ope_node
