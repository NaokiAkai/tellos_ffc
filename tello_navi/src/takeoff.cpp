/****************************************************************************
 * Formation flight controller with Tello EDU
 * Copyright (C) 2022 Naoki Akai
 *
 * Licensed under the Apache License, Version 2.0 (the “License”);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an “AS IS” BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Naoki Akai
 ****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Empty.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "takeoff");
    ros::NodeHandle nh("~");

    std::string takeoffName = "/tello/takeoff";

    nh.param("takeoff_name", takeoffName, takeoffName);

    ros::Publisher takeoffPub = nh.advertise<std_msgs::Empty>(takeoffName, 1);

    std_msgs::Empty msg;
    ros::spinOnce();
    sleep(1);
    takeoffPub.publish(msg);
    ros::spinOnce();
    sleep(1);
    takeoffPub.publish(msg);
    sleep(1);

    return 0;
}