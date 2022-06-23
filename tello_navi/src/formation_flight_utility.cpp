#include <tello_navi/FormationFlightUtility.h>

FormationFlightUtility::FormationFlightUtility(ros::NodeHandle nh, int telloNum):
    nh_(nh),
    telloNum_(telloNum),
    poseTimeTH_(0.5)
{
    telloPoses_.resize(telloNum_);
    targets_.resize(telloNum_);
    poseSubs_.resize(telloNum_);
    targetSubs_.resize(telloNum_);
    takeoffPubs_.resize(telloNum_);
    landPubs_.resize(telloNum_);
    cmdPubs_.resize(telloNum_);
    cmds_.resize(telloNum_);
    poseTimes_.resize(telloNum_);
    targetTimes_.resize(telloNum_);
    cmdTimes_.resize(telloNum_);

    for (int i = 0; i < telloNum_; ++i) {
        std::string poseName = "/vrpn_client_node/tello" + std::to_string(i) + "/pose";
        poseSubs_[i] = nh_.subscribe(poseName, 1, &FormationFlightUtility::poseCB, this);
        poseTimes_[i] = -1.0;

        std::string targetName = "/tello" + std::to_string(i) + "/target";
        targetSubs_[i] = nh_.subscribe(targetName, 1, &FormationFlightUtility::targetCB, this);
        targetTimes_[i] = -1.0;

        std::string takeoffName = "/tello" + std::to_string(i) + "/takeoff";
        takeoffPubs_[i] = nh_.advertise<std_msgs::Empty>(takeoffName, 1);

        std::string landName = "/tello" + std::to_string(i) + "/land";
        landPubs_[i] = nh_.advertise<std_msgs::Empty>(landName, 1);

        std::string cmdName = "/tello" + std::to_string(i) + "/cmd_vel";
        cmdPubs_[i] = nh_.advertise<geometry_msgs::Twist>(cmdName, 1);
        cmdTimes_[i] = -1.0;
    }
}

Pose FormationFlightUtility::geometryMsgsPose2Pose(geometry_msgs::Pose pose) {
    double roll, pitch, yaw;
    tf::Quaternion quat;
    quaternionMsgToTF(pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    Pose p(pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw);
    return p;
}

void FormationFlightUtility::poseCB(const ros::MessageEvent<geometry_msgs::PoseStamped const> &event) {
    std::string topicName = event.getConnectionHeader().at("topic");
    for (int i = 0; i < telloNum_; ++i) {
        std::string name = "/vrpn_client_node/tello" + std::to_string(i) + "/pose";
        if (topicName == name) {
            telloPoses_[i] = geometryMsgsPose2Pose(event.getMessage()->pose);
            poseTimes_[i] = event.getMessage()->header.stamp.toSec();
            break;
        }
    }
}

void FormationFlightUtility::targetCB(const ros::MessageEvent<geometry_msgs::Pose const> &event) {
    std::string topicName = event.getConnectionHeader().at("topic");
    for (int i = 0; i < telloNum_; ++i) {
        std::string name = "/tello" + std::to_string(i) + "/target";
        if (topicName == name) {
            targets_[i] = geometryMsgsPose2Pose(*event.getMessage());
            targetTimes_[i] = ros::Time::now().toSec();
            break;
        }
    }
}

void FormationFlightUtility::takeoff(int id) {
    std_msgs::Empty msg;
    takeoffPubs_[id].publish(msg);
}

void FormationFlightUtility::takeoffAll(void) {
    for (int i = 0; i < telloNum_; ++i)
        takeoff(i);
}

void FormationFlightUtility::land(int id) {
    std_msgs::Empty msg;
    landPubs_[id].publish(msg);
}

void FormationFlightUtility::landAll(void) {
    for (int i = 0; i < telloNum_; ++i)
        land(i);
}

void FormationFlightUtility::publishCmd(geometry_msgs::Twist cmd, int id) {
    // ***********************************************************************************************
    // this velocity conversion operation is implemented due to specification of the tello_driver_node
    double vx = cmd.linear.x;
    double vy = cmd.linear.y;
    cmd.linear.x = -vy;
    cmd.linear.y = vx;
    cmd.angular.z *= -1.0;
    // ***********************************************************************************************
    cmds_[id] = cmd;
    cmdTimes_[id] = ros::Time::now().toSec();
    cmdPubs_[id].publish(cmd);
}

void FormationFlightUtility::publishCmd(double vx, double vy, double vz, double wz, int id) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.linear.z = vz;
    cmd.angular.z = wz;
    publishCmd(cmd, id);
}

bool FormationFlightUtility::getTelloPoses(std::vector<Pose> &poses) {
    double time = ros::Time::now().toSec();
    for (int i = 0; i < telloNum_; ++i) {
        double dt = time - poseTimes_[i];
        if (dt > poseTimeTH_)
            return false;
    }
    poses = telloPoses_;
    return true;
}

bool FormationFlightUtility::getTargets(std::vector<Pose> &targets) {
    for (int i = 0; i < telloNum_; ++i) {
        if (targetTimes_[i] < 0.0)
            return false;
    }
    targets = targets_;
    return true;
}