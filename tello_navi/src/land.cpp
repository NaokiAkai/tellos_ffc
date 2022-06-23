#include <ros/ros.h>
#include <std_msgs/Empty.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "land");
    ros::NodeHandle nh("~");

    std::string landName = "/tello/land";

    nh.param("land_name", landName, landName);

    ros::Publisher landPub = nh.advertise<std_msgs::Empty>(landName, 1);

    std_msgs::Empty msg;
    ros::Rate loopRate(10.0);
    while (ros::ok()) {
        ros::spinOnce();
        landPub.publish(msg);
        loopRate.sleep();
    }
    return 0;
}