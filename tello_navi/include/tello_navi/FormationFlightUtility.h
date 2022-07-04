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

#ifndef __FORMATION_FLIGHT_UTILITY_H___
#define __FORMATION_FLIGHT_UTILITY_H___

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

class Pose {
private:
    double x_, y_, z_;
    double roll_, pitch_, yaw_;

public:
    Pose(void) {}

    Pose(double x, double y, double z, double roll, double pitch, double yaw):
        x_(x), y_(y), z_(z), roll_(roll), pitch_(pitch), yaw_(yaw) {}

    inline void setX(double x) { x_ = x; }
    inline void setY(double y) { y_ = y; }
    inline void setZ(double z) { z_ = z; }
    inline void setRoll(double roll) { roll_ = roll; }
    inline void setPitch(double pitch) { pitch_ = pitch; }
    inline void setYaw(double yaw) { yaw_ = yaw; }
    inline void setPose(Pose p) { x_ = p.getX(), y_ = p.getY(), z_ = p.getZ(), roll_ = p.getRoll(), pitch_ = p.getPitch(), yaw_ = p.getYaw(); }

    inline double getX(void) { return x_; }
    inline double getY(void) { return y_; }
    inline double getZ(void) { return z_; }
    inline double getRoll(void) { return roll_; }
    inline double getPitch(void) { return pitch_; }
    inline double getYaw(void) { return yaw_; }
    inline Pose getPose(void) { return Pose(x_, y_, z_, roll_, pitch_, yaw_); }
}; // class Pose

class FormationFlightUtility {
private:
    ros::NodeHandle nh_;
    int telloNum_;
    std::vector<ros::Subscriber> poseSubs_, targetSubs_;
    std::vector<ros::Publisher> takeoffPubs_, landPubs_, cmdPubs_;
    std::vector<Pose> telloPoses_, targets_;
    std::vector<geometry_msgs::Twist> cmds_;
    std::vector<double> poseTimes_, targetTimes_, cmdTimes_;
    double poseTimeTH_;

public:
    inline int getTelloNum(void) {
        return telloNum_;
    }

    FormationFlightUtility(void) {}

    FormationFlightUtility(ros::NodeHandle nh, int telloNum);

    Pose geometryMsgsPose2Pose(geometry_msgs::Pose pose);

    void poseCB(const ros::MessageEvent<geometry_msgs::PoseStamped const> &event);

    void targetCB(const ros::MessageEvent<geometry_msgs::Pose const> &event);

    void takeoff(int id);

    void takeoffAll(void);

    void land(int id);

    void landAll(void);

    void publishCmd(geometry_msgs::Twist cmd, int id);

    void publishCmd(double vx, double vy, double vz, double wz, int id);

    bool getTelloPoses(std::vector<Pose> &poses);

    bool getTargets(std::vector<Pose> &targets);
}; // class FormationFlightUtility

#endif // __FORMATION_FLIGHT_UTILITY_H___
