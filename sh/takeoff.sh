#! /bin/bash

source ./params
source ../devel/setup.bash

for ((i = 0; i < $tello_num; i++))
do
node_name="tello${i}_takeoff"
takeoff_name="/tello${i}/takeoff"
roslaunch tello_navi takeoff.launch node_name:=$node_name takeoff_name:=$takeoff_name &
done
