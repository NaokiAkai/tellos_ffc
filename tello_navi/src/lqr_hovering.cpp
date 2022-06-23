#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <Eigen/Dense>

class LQRHovering {
private:
    int dimX_, dimU_;
    Eigen::MatrixXd B_, R_, P_;

public:
    LQRHovering(void):
        dimX_(4),
        dimU_(4)
    {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dimX_, dimX_);
        B_ = Eigen::MatrixXd::Zero(dimX_, dimU_);
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dimX_, dimX_);
        R_ = Eigen::MatrixXd::Zero(dimU_, dimU_);
        P_ = Eigen::MatrixXd::Zero(dimX_, dimX_);

        A(0, 0) = A(0, 1) = A(0, 2) = A(0,3) = 0.0;
        A(1, 0) = A(1, 1) = A(1, 2) = A(1,3) = 0.0;
        A(2, 0) = A(2, 1) = A(2, 2) = A(2,3) = 0.0;
        A(3, 0) = A(3, 1) = A(3, 2) = A(3,3) = 0.0;

        B_(0, 0) = B_(1, 1) = B_(2, 2) = B_(3, 3) = 1.0;
        B_(0, 1) = B_(0, 2) = B_(0, 3) = 0.0;
        B_(1, 0) = B_(1, 2) = B_(1, 3) = 0.0;
        B_(2, 0) = B_(2, 1) = B_(2, 3) = 0.0;
        B_(3, 0) = B_(3, 1) = B_(3, 2) = 0.0;

        Q(0, 0) = 1.0;
        Q(1, 1) = 1.0;
        Q(2, 2) = 1.0;
        Q(3, 3) = 1.0;
        Q(0, 1) = Q(0, 2) = Q(0, 3) = 0.0;
        Q(1, 0) = Q(1, 2) = Q(1, 3) = 0.0;
        Q(2, 0) = Q(2, 1) = Q(2, 3) = 0.0;
        Q(3, 0) = Q(3, 1) = Q(3, 2) = 0.0;

        R_(0, 0) = 1.0;
        R_(1, 1) = 1.0;
        R_(2, 2) = 1.0;
        R_(3, 3) = 1.0;
        R_(0, 1) = R_(0, 2) = R_(0, 3) = 0.0;
        R_(1, 0) = R_(1, 2) = R_(1, 3) = 0.0;
        R_(2, 0) = R_(2, 1) = R_(2, 3) = 0.0;
        R_(3, 0) = R_(3, 1) = R_(3, 2) = 0.0;

        if (!solveRiccatiArimotoPotter(A, B_, Q, R_, P_)) {
            fprintf(stderr, "Could not solve Riccati.\n");
            exit(1);
        }
    }

    Eigen::VectorXd solve(double poseX, double poseY, double poseZ, double poseYaw,
        double targetX, double targetY, double targetZ, double targetYaw)
    {
        Eigen::VectorXd e = Eigen::VectorXd::Zero(dimX_);
        e(0) = poseX - targetX;
        e(1) = poseY - targetY;
        e(2) = poseZ - targetZ;
        e(3) = poseYaw - targetYaw;
        while (e(3) < -M_PI)
            e(3) += 2.0 * M_PI;
        while (e(3) > M_PI)
            e(3) -= 2.0 * M_PI;

        Eigen::VectorXd u = Eigen::VectorXd::Zero(dimU_);
        u = -R_.inverse() * B_.transpose() * P_ * e;
        return u;
    }

private:
    bool solveRiccatiArimotoPotter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
        const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, Eigen::MatrixXd &P)
    {
        const uint dimX = A.rows();
        const uint dimU = B.cols();

        Eigen::MatrixXd Ham = Eigen::MatrixXd::Zero(2 * dimX, 2 * dimX);
        Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

        Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);
        Eigen::MatrixXcd eigvec = Eigen::MatrixXcd::Zero(2 * dimX, dimX);
        int j = 0;
        for (int i = 0; i < 2 * dimX; ++i) {
            if (Eigs.eigenvalues()[i].real() < 0.) {
                eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * dimX, 1);
                ++j;
            }
        }

        Eigen::MatrixXcd Vs_1, Vs_2;
        Vs_1 = eigvec.block(0, 0, dimX, dimX);
        Vs_2 = eigvec.block(dimX, 0, dimX, dimX);
        P = (Vs_2 * Vs_1.inverse()).real();

        return true;
    }

    void printMatrix(Eigen::MatrixXcd X) {
        std::cout << X << std::endl << std::endl;
    }
}; // class LQRHovering

class LQRHoveringNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber poseSub_, targetSub_;
    ros::Publisher cmdPub_;
    std::string poseName_, targetName_, cmdName_;
    double poseX_, poseY_, poseZ_, poseYaw_;
    double targetX_, targetY_, targetZ_, targetYaw_;
    double poseTime_, targetTime_;
    double controlHz_;
    double maxLinearVel_, maxAngularVel_;
    LQRHovering lqr_;

public:
    LQRHoveringNode(void):
        nh_("~"),
        poseName_("/tello/pose"),
        targetName_("/tello/target"),
        cmdName_("/tello/cmd_vel"),
        targetX_(0.0),
        targetY_(0.0),
        targetZ_(0.0),
        targetYaw_(0.0),
        controlHz_(30.0),
        maxLinearVel_(0.5),
        maxAngularVel_(0.5),
        poseTime_(-1.0),
        targetTime_(ros::Time::now().toSec())
    {
        nh_.param("pose_name", poseName_, poseName_);
        nh_.param("target_name", targetName_, targetName_);
        nh_.param("cmd_name", cmdName_, cmdName_);
        nh_.param("target_x", targetX_, targetX_);
        nh_.param("target_y", targetY_, targetY_);
        nh_.param("target_z", targetZ_, targetZ_);
        nh_.param("target_yaw", targetYaw_, targetYaw_);
        nh_.param("control_hz", controlHz_, controlHz_);
        nh_.param("max_linear_vel", maxLinearVel_, maxLinearVel_);
        nh_.param("max_angular_vel", maxAngularVel_, maxAngularVel_);

        poseSub_ = nh_.subscribe(poseName_, 1, &LQRHoveringNode::poseCB, this);
        targetSub_ = nh_.subscribe(targetName_, 1, &LQRHoveringNode::targetCB, this);

        cmdPub_ = nh_.advertise<geometry_msgs::Twist>(cmdName_, 1);
    }

    void spin(void) {
        ros::Rate loopRate(controlHz_);
        while (ros::ok()) {
            ros::spinOnce();
            if (!isValidPoseTimestamp()) {
                ROS_ERROR("pose message might not be published.");
                publishCmd(0.0, 0.0, 0.0, 0.0);
            } else {
                Eigen::VectorXd u = lqr_.solve(poseX_, poseY_, poseZ_, poseYaw_, targetX_, targetY_, targetZ_, targetYaw_);
                publishCmd(u(0), u(1), u(2), u(3));
            }
            loopRate.sleep();
        }
    }

private:
    void poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        poseTime_ = msg->header.stamp.toSec();
        poseX_ = msg->pose.position.x;
        poseY_ = msg->pose.position.y;
        poseZ_ = msg->pose.position.z;

        double roll, pitch;
        tf::Quaternion quat;
        quaternionMsgToTF(msg->pose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, poseYaw_);
    }

    void targetCB(const geometry_msgs::Pose::ConstPtr &msg) {
        targetTime_ = ros::Time::now().toSec();
        targetX_ = msg->position.x;
        targetY_ = msg->position.y;
        targetZ_ = msg->position.z;

        double roll, pitch;
        tf::Quaternion quat;
        quaternionMsgToTF(msg->orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, targetYaw_);
    }

    bool isValidPoseTimestamp(void) {
        double dt = ros::Time::now().toSec() - poseTime_;
        if (dt > 1.0)
            return false;
        else
            return true;
    }

    void publishCmd(double vx, double vy, double vz, double wz) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = clipVelocity(vx, maxLinearVel_);
        cmd.linear.y = clipVelocity(vy, maxLinearVel_);
        cmd.linear.z = clipVelocity(vz, maxLinearVel_);
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = clipVelocity(wz, maxAngularVel_);
        cmdPub_.publish(cmd);
        ROS_INFO("vx = %lf, vy = %lf, vz = %lf, wz = %lf", cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z);
    }

    double clipVelocity(double v, double vMax) {
        if (v > vMax)
            return vMax;
        else if (v < -vMax)
            return -vMax;
        else
            return v;
    }
}; // class LQRHoveringNode

int main(int argc, char **argv) {
    ros::init(argc, argv, "lqr_hovering");
    LQRHoveringNode node;
    node.spin();
    return 0;
}