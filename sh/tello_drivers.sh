#! /bin/bash

source ./params
source ../../../devel/setup.bash

for ((i = 0; i < $tello_num; i++))
do
name_space="tello${i}"
roslaunch tello_driver tello_node.launch namespace:=$name_space tello_ip:=${tello_ip[i]} &
done

sleep 3
echo -e "\n\n"
read -p "Hit enter to shutdown the drivers: "

for ((i = 0; i < $tello_num; i++))
do
driver_node_name="/tello${i}/tello_driver_node"
image_node_name="/tello${i}/image_compressed"
rosnode kill $driver_node_name
rosnode kill $image_node_name
done
