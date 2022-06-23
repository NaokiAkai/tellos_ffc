#! /bin/bash

source ./params
source ../devel/setup.bash

for ((i = 0; i < $tello_num; i++))
do
node_name="tello${i}_sim"
takeoff_name="/tello${i}/takeoff"
land_name="/tello${i}/land"
cmd_name="/tello${i}/cmd_vel"
pose_name="/vrpn_client_node/tello${i}/pose"
base_frame="base_link${i}"
roslaunch tello_navi simple_tello_sim.launch node_name:=$node_name takeoff_name:=$takeoff_name land_name:=$land_name cmd_name:=$cmd_name pose_name:=$pose_name base_frame:=$base_frame initial_x:=${initial_x[i]} initial_y:=${initial_y[i]} &
done

roslaunch tello_navi tf_markers.launch tello_num:=$tello_num &
sleep 2

./target_sender.sh &
./takeoff.sh

sleep 3
echo -e "\n\n"
read -p "Hit enter to shutdown the simulators: "

for ((i = 0; i < $tello_num; i++))
do
node_name="/tello${i}_sim"
rosnode kill $node_name
done
rosnode kill tf_markers
