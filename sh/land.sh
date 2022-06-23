#! /bin/bash

source ./params
source ../devel/setup.bash

for ((i = 0; i < $tello_num; i++))
do
node_name="tello${i}_land"
land_name="/tello${i}/land" 
roslaunch tello_navi land.launch node_name:=$node_name land_name:=$land_name &
done

sleep 3
echo -e "\n\n"
read -p "Hit enter to shutdown the land programs: "

for ((i = 0; i < $tello_num; i++))
do
node_name="/tello${i}_land"
rosnode kill $node_name
done
