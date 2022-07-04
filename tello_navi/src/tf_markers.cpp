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
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

class TFMarkers {
private:
    ros::NodeHandle nh_;
    ros::Publisher markersPub_;
    int telloNum_;
    std::string markersName_;
    double propellerSize_, propellerScale_, shaftLength_, shaftScale_;

public:
    TFMarkers(void):
        nh_("~"),
        telloNum_(6),
        markersName_("/tellos_marker"),
        propellerSize_(0.3),
        propellerScale_(0.05),
        shaftLength_(0.6),
        shaftScale_(0.05)
    {
        nh_.param("tello_num", telloNum_, telloNum_);
        nh_.param("markers_name", markersName_, markersName_);
        nh_.param("propeller_size", propellerSize_, propellerSize_);
        nh_.param("propeller_scale", propellerScale_, propellerScale_);
        nh_.param("shaft_length", shaftLength_, shaftLength_);
        nh_.param("shaft_scale", shaftScale_, shaftScale_);

        markersPub_ = nh_.advertise<visualization_msgs::MarkerArray>(markersName_, 1);
    }

    void spin(void) {
        ros::Rate loopRate(20.0);
        while (ros::ok()) {
            visualization_msgs::MarkerArray markers;
            int markerID = 0;
            for (int i = 0; i < telloNum_; ++i) {
                visualization_msgs::Marker propeller;
                std::string baseFrame = "base_link" + std::to_string(i);
                propeller.header.frame_id = baseFrame;
                propeller.header.stamp = ros::Time::now();
                propeller.ns = "tello_propellers";
                propeller.lifetime = ros::Duration();
                propeller.frame_locked = true;
                propeller.type = visualization_msgs::Marker::CYLINDER;
                propeller.action = visualization_msgs::Marker::ADD;
                propeller.color.r = 0.75294117647;
                propeller.color.g = 0.18823529411;
                propeller.color.b = 0.75294117647;
                propeller.color.a = 1.0;
                propeller.scale.x = propellerSize_;
                propeller.scale.y = propellerSize_;
                propeller.scale.z = propellerScale_;
                propeller.pose.orientation.x = 0.0;
                propeller.pose.orientation.y = 0.0;
                propeller.pose.orientation.z = 0.0;
                propeller.pose.orientation.w = 1.0;
                for (int i = 0; i < 4; ++i) {
                    propeller.id = markerID;
                    propeller.pose.position.x = shaftLength_ / 2.0 * cos(i * M_PI / 2.0 + M_PI / 4.0);
                    propeller.pose.position.y = shaftLength_ / 2.0 * sin(i * M_PI / 2.0 + M_PI / 4.0);
                    propeller.pose.position.z = 0.0;
                    markers.markers.push_back(propeller);
                    markerID++;
                }

                visualization_msgs::Marker shafts;
                shafts.header.frame_id = baseFrame;
                shafts.header.stamp = ros::Time::now();
                shafts.ns = "tello_shafts";
                shafts.lifetime = ros::Duration();
                shafts.frame_locked = true;
                shafts.type = visualization_msgs::Marker::LINE_LIST;
                shafts.action = visualization_msgs::Marker::ADD;
                shafts.color.r = 0.75294117647;
                shafts.color.g = 0.18823529411;
                shafts.color.b = 0.75294117647;
                shafts.color.a = 1.0;
                shafts.scale.x = shaftScale_;
                shafts.scale.y = 0.0;
                shafts.scale.z = 0.0;
                shafts.pose.orientation.x = 0.0;
                shafts.pose.orientation.y = 0.0;
                shafts.pose.orientation.z = 0.0;
                shafts.pose.orientation.w = 1.0;
                shafts.id = markerID;
                for (int i = 0; i < 2; ++i) {
                    geometry_msgs::Point p1, p2;
                    p1.x = shaftLength_ / 2.0 * cos(i * M_PI / 2.0 + M_PI / 4.0);
                    p1.y = shaftLength_ / 2.0 * sin(i * M_PI / 2.0 + M_PI / 4.0);
                    p2.x = shaftLength_ / 2.0 * cos(i * M_PI / 2.0 + M_PI / 4.0 + M_PI);
                    p2.y = shaftLength_ / 2.0 * sin(i * M_PI / 2.0 + M_PI / 4.0 + M_PI);
                    p1.z = p2.z = 0.0;
                    shafts.points.push_back(p1);
                    shafts.points.push_back(p2);
                }
                markerID++;

                markers.markers.push_back(shafts);
            }
            markersPub_.publish(markers);
            loopRate.sleep();
        }
    }
}; // class TFMarkers

int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_markers");
    TFMarkers node;
    node.spin();
    return 0;
}
