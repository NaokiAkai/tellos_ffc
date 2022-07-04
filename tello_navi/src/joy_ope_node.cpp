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
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tello_navi/FormationFlightUtility.h>

class JoyOpeNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber joySub_;
    geometry_msgs::Twist cmd_;
    int telloNum_, targetTelloNum_;
    FormationFlightUtility *utility_;
    double linearVelCoef_, angularVelCoef_;

public:
    JoyOpeNode(void):
        nh_("~"),
        telloNum_(6),
        linearVelCoef_(1.0),
        angularVelCoef_(1.0),
        targetTelloNum_(0)
    {
        nh_.param("tello_Num", telloNum_, telloNum_);
        nh_.param("linear_vel_coef", linearVelCoef_, linearVelCoef_);
        nh_.param("angular_vel_coef", angularVelCoef_, angularVelCoef_);

        joySub_ = nh_.subscribe("/joy", 1, &JoyOpeNode::joyCB, this);

        utility_ = new FormationFlightUtility(nh_, telloNum_);

        cmd_.linear.x = cmd_.linear.y = cmd_.linear.z = cmd_.angular.z = 0.0;
    }

    void spin(void) {
        ros::Rate loopRate(10.0);
        while (ros::ok()) {
            ros::spinOnce();
            utility_->publishCmd(cmd_, targetTelloNum_);
            ROS_INFO("id: = %d, vx = %lf, vy = %lf, vz = %lf, wz = %lf",
                targetTelloNum_, cmd_.linear.x, cmd_.linear.y, cmd_.linear.z, cmd_.angular.z);
            loopRate.sleep();
        }
    }

    void joyCB(const sensor_msgs::Joy::ConstPtr &msg) {
        if (msg->buttons[2] == 1) {
            utility_->takeoffAll();
            ROS_INFO("takeoff");
        } else if (msg->buttons[0] == 1) {
            utility_->landAll();
            ROS_INFO("land");
        } else if (msg->buttons[5] == 1) {
            targetTelloNum_++;
            if (targetTelloNum_ >= telloNum_)
                targetTelloNum_ = 0;
        } else {
/*
            cmd_.linear.x = -msg->axes[0] * linearVelCoef_;
            cmd_.linear.y = msg->axes[1] * linearVelCoef_;
            cmd_.linear.z = msg->axes[4] * linearVelCoef_;
            cmd_.angular.z = -msg->axes[3] * angularVelCoef_;
 */
            cmd_.linear.x = msg->axes[1] * linearVelCoef_;
            cmd_.linear.y = msg->axes[0] * linearVelCoef_;
            cmd_.linear.z = msg->axes[4] * linearVelCoef_;
            cmd_.angular.z = msg->axes[3] * angularVelCoef_;
        }
    }
}; // class JoyOpeNode

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_ope_node");
    JoyOpeNode node;
    node.spin();
    return 0;
}