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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class SimpleTelloSim {
private:
    ros::NodeHandle nh_;
    ros::Subscriber takeoffSub_, landSub_, cmdSub_;
    ros::Publisher posePub_;
    std::string mapFrame_, baseFrame_, takeoffName_, landName_, cmdName_, poseName_;
    geometry_msgs::Twist cmd_;
    double lastCmdTime_;
    double poseX_, poseY_, poseZ_, poseYaw_, vx_, vy_, vz_, wz_, groundZ_;
    double simHz_, systemGain_, timeConstant_;
    bool canUpdateCmd_;
    tf::TransformBroadcaster tfBroadcaster_;

public:
    SimpleTelloSim(void):
        nh_("~"),
        mapFrame_("map"),
        baseFrame_("base_link"),
        takeoffName_("/tello/takeoff"),
        landName_("/tello/land"),
        cmdName_("/tello/cmd_vel"),
        poseName_("/vrpn_client_ros/tello/pose"),
        poseX_(0.0),
        poseY_(0.0),
        poseZ_(0.0),
        poseYaw_(0.0),
        vx_(0.0),
        vy_(0.0),
        vz_(0.0),
        wz_(0.0),
        simHz_(20.0),
        systemGain_(3.0),
        timeConstant_(0.2),
        lastCmdTime_(-1.0),
        tfBroadcaster_(),
        canUpdateCmd_(false)
    {
        nh_.param("map_frame", mapFrame_, mapFrame_);
        nh_.param("base_frame", baseFrame_, baseFrame_);
        nh_.param("takeoff_name", takeoffName_, takeoffName_);
        nh_.param("land_name", landName_, landName_);
        nh_.param("cmd_name", cmdName_, cmdName_);
        nh_.param("pose_name", poseName_, poseName_);
        nh_.param("initial_x", poseX_, poseX_);
        nh_.param("initial_y", poseY_, poseY_);
        nh_.param("initial_z", poseZ_, poseZ_);
        nh_.param("initial_yaw", poseYaw_, poseYaw_);
        nh_.param("sim_hz", simHz_, simHz_);
        nh_.param("system_gain", systemGain_, systemGain_);
        nh_.param("time_constant", timeConstant_, timeConstant_);

        groundZ_ = poseZ_;

        takeoffSub_ = nh_.subscribe(takeoffName_, 1, &SimpleTelloSim::takeoffCB, this);
        landSub_ = nh_.subscribe(landName_, 1, &SimpleTelloSim::landCB, this);
        cmdSub_ = nh_.subscribe(cmdName_, 1, &SimpleTelloSim::cmdCB, this);

        posePub_ = nh_.advertise<geometry_msgs::PoseStamped>(poseName_, 1);
    }

    void takeoffCB(const std_msgs::Empty::ConstPtr &msg) {
        canUpdateCmd_ = true;
        poseZ_ = groundZ_ + 1.0;
        ROS_INFO("takeoff");
    }

    void landCB(const std_msgs::Empty::ConstPtr &msg) {
        canUpdateCmd_ = false;
        poseZ_ = groundZ_;
        cmd_.linear.x = cmd_.linear.y = cmd_.linear.z = cmd_.angular.z = 0.0;
        vx_ = vy_ = vz_ = wz_ = 0.0;
        ROS_INFO("land");
    }

    void cmdCB(const geometry_msgs::Twist::ConstPtr &msg) {
        if (canUpdateCmd_) {
            cmd_ = *msg;
            lastCmdTime_ = ros::Time::now().toSec();
        }
    }

    void updatePose(void) {
        static bool isFirst = true;
        static double prevTime;
        double currTime = ros::Time::now().toSec();
        if (isFirst) {
            prevTime = currTime;
            isFirst = false;
            return;
        }

        double dt = currTime - prevTime;
        double a = -timeConstant_ / (timeConstant_ + dt);
        double b = systemGain_ * dt / (timeConstant_ + dt);

        // *************************************************************************************
        // this velocity conversion is implemented due to specification of the tello_driver_node
        vx_ = a * vx_ + b * cmd_.linear.y;
        vy_ = a * vy_ + b * (-cmd_.linear.x);
        vz_ = a * vz_ + b * cmd_.linear.z;
        wz_ = a * wz_ + b * (-cmd_.angular.z);
        // *************************************************************************************

        double c = cos(poseYaw_);
        double s = sin(poseYaw_);
        poseX_ += (vx_ * c - vy_ * s) * dt;
        poseY_ += (vx_ * s + vy_ * c) * dt;
        poseZ_ += vz_ * dt;
        poseYaw_ += wz_ * dt;
        while (poseYaw_ < -M_PI)
            poseYaw_ += 2.0 * M_PI;
        while (poseYaw_ > M_PI)
            poseYaw_ -= 2.0 * M_PI;

        prevTime = currTime;
    }

    void broadcastTF(void) {
        geometry_msgs::Pose pose;
        pose.position.x = poseX_;
        pose.position.y = poseY_;
        pose.position.z = poseZ_;
        pose.orientation = tf::createQuaternionMsgFromYaw(poseYaw_);

        tf2::Transform map2baseTrans;
        tf2::convert(pose, map2baseTrans);

        geometry_msgs::TransformStamped map2baseTransStamped;
        map2baseTransStamped.header.frame_id = mapFrame_;
        map2baseTransStamped.header.stamp = ros::Time::now();
        map2baseTransStamped.child_frame_id = baseFrame_;
        tf2::convert(map2baseTrans, map2baseTransStamped.transform);
        tfBroadcaster_.sendTransform(map2baseTransStamped);
    }

    void publishPose(void) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = mapFrame_;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = poseX_;
        pose.pose.position.y = poseY_;
        pose.pose.position.z = poseZ_;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(poseYaw_);
        posePub_.publish(pose);
    }

    void printInfo(void) {
        ROS_INFO("x = %lf, y = %lf, z = %lf, yaw = %lf", poseX_, poseY_, poseZ_, poseYaw_);
    }

    void spin(void) {
        ros::Rate loopRate(simHz_);
        while (ros::ok()) {
            ros::spinOnce();
            updatePose();
            broadcastTF();
            publishPose();
            printInfo();
            loopRate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_tello_sim");
    SimpleTelloSim node;
    node.spin();
    return 0;
}