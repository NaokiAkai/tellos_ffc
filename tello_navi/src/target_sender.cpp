#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

class TargetSender {
private:
    ros::NodeHandle nh_;
    int telloNum_;
    std::vector<ros::Publisher> targetPubs_;
    ros::Publisher markerPub_;
    ros::Subscriber moveTargetsSub_;
    std::string mapFrame_, markerName_;
    bool moveTargets_;
    double sendHz_, startTime_;

public:
    TargetSender(void):
        nh_("~"),
        telloNum_(6),
        mapFrame_("map"),
        markerName_("/target_markers"),
        sendHz_(30.0),
        moveTargets_(false)
    {
        nh_.param("send_hz", sendHz_, sendHz_);
        nh_.param("marker_name", markerName_, markerName_);

        targetPubs_.resize(telloNum_);
        for (int i = 0; i < telloNum_; ++i) {
            std::string name = "/tello" + std::to_string(i) + "/target";
            targetPubs_[i] = nh_.advertise<geometry_msgs::Pose>(name, 1);
        }
        markerPub_ = nh_.advertise<visualization_msgs::Marker>(markerName_, 1);

        moveTargetsSub_ = nh_.subscribe("/move_targets", 1, &TargetSender::moveTargetsCB, this);

        startTime_ = ros::Time::now().toSec();
    }

    void moveTargetsCB(const std_msgs::Empty::ConstPtr &msg) {
        if (moveTargets_) {
            moveTargets_ = false;
        } else {
            moveTargets_ = true;
            startTime_ = ros::Time::now().toSec();
            ROS_INFO("Move targets");
        }
    }

    void spin(void) {
        ros::Rate loopRate(sendHz_);

        visualization_msgs::Marker marker;
        marker.header.frame_id = mapFrame_;
        marker.id = 0;
        marker.header.stamp = ros::Time::now();
        marker.ns = "target_markers";
        marker.lifetime = ros::Duration();
        marker.frame_locked = true;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        while (ros::ok()) {
            ros::spinOnce();
            double t = 0.1 * (ros::Time::now().toSec() - startTime_);
            marker.points.clear();

            geometry_msgs::Point point;
            geometry_msgs::Pose targetLeader;
            geometry_msgs::Pose targetSub;

            // for leader
/*
            targetLeader.position.x = 1.0 * cos(t);
            targetLeader.position.y = 1.0 * sin(t);
            targetLeader.position.z = 1.5;
 */
            targetLeader.position.x = 1.0;
            targetLeader.position.y = 0.0;
            targetLeader.position.z = 1.5;
            if (moveTargets_) {
                targetLeader.position.x = 1.0 + 1.0 * cos(t);
                targetLeader.position.y = 0.0 + 1.0 * sin(t);
                targetLeader.position.z = 1.5;
            }
            tf::Quaternion quatLeader = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
            tf::quaternionTFToMsg(quatLeader, targetLeader.orientation);
/*
            targetLeader.orientation.x = quatLeader.x;
            targetLeader.orientation.y = quatLeader.y;
            targetLeader.orientation.z = quatLeader.z;
            targetLeader.orientation.w = quatLeader.w;
 */
            targetPubs_[0].publish(targetLeader);
            point.x = targetLeader.position.x;
            point.y = targetLeader.position.y;
            point.z = targetLeader.position.z;
            marker.points.push_back(point);


            // for sub1
            targetSub.position.x = 0.0;
            targetSub.position.y = 0.5;
            targetSub.position.z = 1.5;
            if (moveTargets_) {
                targetSub.position.x = 0.0 + 1.0 * cos(t);
                targetSub.position.y = 0.5 + 1.0 * sin(t);
                targetSub.position.z = 1.5;
            }
/*
            targetSub.orientation.x = 0.0;
            targetSub.orientation.y = 0.0;
            targetSub.orientation.z = 0.0;
            targetSub.orientation.w = 1.0;
 */
            targetSub.orientation = targetLeader.orientation;
            targetPubs_[1].publish(targetSub);
            point.x = targetSub.position.x;
            point.y = targetSub.position.y;
            point.z = targetSub.position.z;
            marker.points.push_back(point);

            // for sub2
            targetSub.position.x = 0.0;
            targetSub.position.y = -0.5;
            targetSub.position.z = 1.5;
            if (moveTargets_) {
                targetSub.position.x = 0.0 + 1.0 * cos(t);
                targetSub.position.y = -0.5 + 1.0 * sin(t);
                targetSub.position.z = 1.5;
            }
/*
            targetSub.orientation.x = 0.0;
            targetSub.orientation.y = 0.0;
            targetSub.orientation.z = 0.0;
            targetSub.orientation.w = 1.0;
 */
            targetSub.orientation = targetLeader.orientation;
            targetPubs_[2].publish(targetSub);
            point.x = targetSub.position.x;
            point.y = targetSub.position.y;
            point.z = targetSub.position.z;
            marker.points.push_back(point);

            // for sub3
            targetSub.position.x = -1.0;
            targetSub.position.y = 1.0;
            targetSub.position.z = 1.5;
            if (moveTargets_) {
                targetSub.position.x = -1.0 + 1.0 * cos(t);
                targetSub.position.y = 1.0 + 1.0 * sin(t);
                targetSub.position.z = 1.5;
            }
/*
            targetSub.orientation.x = 0.0;
            targetSub.orientation.y = 0.0;
            targetSub.orientation.z = 0.0;
            targetSub.orientation.w = 1.0;
 */
            targetSub.orientation = targetLeader.orientation;
            targetPubs_[3].publish(targetSub);
            point.x = targetSub.position.x;
            point.y = targetSub.position.y;
            point.z = targetSub.position.z;
            marker.points.push_back(point);

            // for sub4
            targetSub.position.x = -1.0;
            targetSub.position.y = 0.0;
            targetSub.position.z = 1.5;
            if (moveTargets_) {
                targetSub.position.x = -1.0 + 1.0 * cos(t);
                targetSub.position.y = 0.0 + 1.0 * sin(t);
                targetSub.position.z = 1.5;
            }
/*
            targetSub.orientation.x = 0.0;
            targetSub.orientation.y = 0.0;
            targetSub.orientation.z = 0.0;
            targetSub.orientation.w = 1.0;
 */
            targetSub.orientation = targetLeader.orientation;
            targetPubs_[4].publish(targetSub);
            point.x = targetSub.position.x;
            point.y = targetSub.position.y;
            point.z = targetSub.position.z;
            marker.points.push_back(point);

            // for sub5
            targetSub.position.x = -1.0;
            targetSub.position.y = -1.0;
            targetSub.position.z = 1.5;
            if (moveTargets_) {
                targetSub.position.x = -1.0 + 1.0 * cos(t);
                targetSub.position.y = -1.0 + 1.0 * sin(t);
                targetSub.position.z = 1.5;
            }
/*
            targetSub.orientation.x = 0.0;
            targetSub.orientation.y = 0.0;
            targetSub.orientation.z = 0.0;
            targetSub.orientation.w = 1.0;
 */
            targetSub.orientation = targetLeader.orientation;
            targetPubs_[5].publish(targetSub);
            point.x = targetSub.position.x;
            point.y = targetSub.position.y;
            point.z = targetSub.position.z;
            marker.points.push_back(point);

            markerPub_.publish(marker);

            loopRate.sleep();
        }
    }
}; // class TargetSender

int main(int argc, char **argv) {
    ros::init(argc, argv, "target_sender");
    TargetSender node;
    node.spin();
    return 0;
}