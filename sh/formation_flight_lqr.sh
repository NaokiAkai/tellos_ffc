#! /bin/bash

source ./params
source ../devel/setup.bash

q_ind=0.3
q_cp=0.1
max_linear_vel=1.0
max_angular_vel=1.0

roslaunch tello_navi formation_flight_lqr.launch q_ind:=$q_ind q_cp:=$q_cp max_linear_vel:=$max_linear_vel max_angular_vel:=$max_angular_vel

rosnode kill /formation_flight_lqr
