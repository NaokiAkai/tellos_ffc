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