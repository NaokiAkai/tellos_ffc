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
#include <tello_navi/LQR.h>
#include <tello_navi/FormationFlightUtility.h>
#include <Eigen/Dense>

class FormationFlightLQR {
private:
    ros::NodeHandle nh_;
    LQR lqr_;
    FormationFlightUtility *utility_;
    double controlHz_;
    double qInd_, qCp_;
    double maxLinearVel_, maxAngularVel_;
    int telloNum_, dimX_, dimU_;

    Eigen::MatrixXd KroneckerProduct(Eigen::MatrixXd A, Eigen::MatrixXd B) {
        Eigen::MatrixXd C(A.rows() * B.rows(), A.cols() * B.cols());
        for (int i1 = 0; i1 < A.rows(); ++i1) {
            for (int j1 = 0; j1 < A.cols(); ++j1) {
                for (int i2 = 0; i2 < B.rows(); ++i2) {
                    for (int j2 = 0; j2 < B.cols(); ++j2)
                        C(i1 * B.rows() + i2, j1 * B.cols() + j2) = A(i1, j1) * B(i2, j2);
                }
            }
        }
        return C;
    }

public:
    FormationFlightLQR(void):
        nh_("~"),
        telloNum_(6),
        controlHz_(30.0),
        qInd_(1.0),
        qCp_(1.0),
        maxLinearVel_(1.0),
        maxAngularVel_(1.0)
    {
        nh_.param("tello_num", telloNum_, telloNum_);
        nh_.param("cotrol_hz", controlHz_, controlHz_);
        nh_.param("q_ind", qInd_, qInd_);
        nh_.param("q_cp", qCp_, qCp_);
        nh_.param("max_linear_vel", maxLinearVel_, maxLinearVel_);
        nh_.param("max_angular_vel", maxAngularVel_, maxAngularVel_);

        utility_ = new FormationFlightUtility(nh_, telloNum_);

        dimX_ = 4 * telloNum_;
        dimU_ = 4 * telloNum_;

        Eigen::MatrixXd Lstr = Eigen::MatrixXd::Zero(telloNum_, telloNum_);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(telloNum_, telloNum_);
        for (int i = 0; i < telloNum_; ++i) {
            for (int j = 0; j < telloNum_; ++j) {
                if (i == 0 && j == 0) {
                    Lstr(i, j) = telloNum_ - 1;
                } else if (i == 0 || j == 0) {
                    Lstr(i, j) = -1.0;
                } else if (i == j) {
                    Lstr(i, j) = 1.0;
                } else {
                    Lstr(i, j) = 0.0;
                }
            }
        }

        Eigen::MatrixXd Cind = Eigen::MatrixXd::Zero(4, 4);
        Eigen::MatrixXd Ccp = Eigen::MatrixXd::Zero(4, 4);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                if (i == j) {
                    Cind(i, j) = 1.0;
                    Ccp(i, j) = 1.0;
                } else {
                    Cind(i, j) = 0.0;
                    Ccp(i, j) = 0.0;
                }
            }
        }

        Cind = Cind.transpose() * Cind;
        Ccp = Ccp.transpose() * Ccp;

        Eigen::MatrixXd Qind = KroneckerProduct(I, Cind);
        Eigen::MatrixXd Qcp = KroneckerProduct(Lstr, Ccp);

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dimX_, dimX_);
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(dimX_, dimU_);
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dimX_, dimX_);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dimU_, dimU_);
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dimX_, dimX_);

        for (int i = 0; i < dimX_; ++i) {
            for (int j = 0; j < dimX_; ++j)
                A(i, j) = 0.0;
        }

        for (int i = 0; i < dimX_; ++i) {
            for (int j = 0; j < dimU_; ++j) {
                if (i == j)
                    B(i, j) = 1.0;
                else
                    B(i, j) = 0.0;
            }
        }

        Q = qInd_ * Qind + qCp_ * Qcp;

        for (int i = 0; i < dimU_; ++i) {
            for (int j = 0; j < dimU_; ++j)
                if (i == j)
                    R(i, j) = 1.0;
                else
                    R(i, j) = 0.0;
        }

        lqr_ = LQR(A, B, Q, R);
//        lqr_.printFeedbackGain();
    }

    Eigen::VectorXd getOutput(std::vector<Pose> telloPoses, std::vector<Pose> targets) {
        Eigen::VectorXd output(dimX_);
        for (int i = 0; i < telloNum_; ++i) {
            output(4 * i + 0) = telloPoses[i].getX() - targets[i].getX();
            output(4 * i + 1) = telloPoses[i].getY() - targets[i].getY();
            output(4 * i + 2) = telloPoses[i].getZ() - targets[i].getZ();
            output(4 * i + 3) = telloPoses[i].getYaw() - targets[i].getYaw();
            while (output(4 * i + 3) < -M_PI)
                output(4 * i + 3) += 2.0 * M_PI;
            while (output(4 * i + 3) > M_PI)
                output(4 * i + 3) -= 2.0 * M_PI;
        }
        return output;
    }

    double clipVelocity(double v, double vMax) {
        if (v > vMax)
            return vMax;
        else if (v < -vMax)
            return -vMax;
        else
            return v;
    }

    bool checkNaN(Eigen::VectorXd controlInput) {
        for (int i = 0; i < (int)controlInput.size(); ++i) {
            if (std::isnan(controlInput(i)))
                return true;
        }
        return false;
    }

    Eigen::VectorXd modifyControlInputAccordingToYaw(Eigen::VectorXd controlInput, std::vector<Pose> telloPoses) {
        for (int i = 0; i < telloNum_; ++i) {
            double ux = controlInput(4 * i + 0);
            double uy = controlInput(4 * i + 1);
            double yaw = telloPoses[i].getYaw();
            controlInput(4 * i + 0) = ux * cos(yaw) + uy * sin(yaw);
            controlInput(4 * i + 1) = -ux * sin(yaw) + uy * cos(yaw);
        }
        return controlInput;
    }

    Eigen::VectorXd clipVelocities(Eigen::VectorXd controlInput) {
        for (int i = 0; i < telloNum_; ++i) {
            controlInput(4 * i + 0) = clipVelocity(controlInput(4 * i + 0), maxLinearVel_);
            controlInput(4 * i + 1) = clipVelocity(controlInput(4 * i + 1), maxLinearVel_);
            controlInput(4 * i + 2) = clipVelocity(controlInput(4 * i + 2), maxLinearVel_);
            controlInput(4 * i + 3) = clipVelocity(controlInput(4 * i + 3), maxAngularVel_);
        }
        return controlInput;
    }

    void spin(void) {
        ros::Rate loopRate(controlHz_);
        while (ros::ok()) {
            ros::spinOnce();
            std::vector<Pose> telloPoses, targets;
            bool emergency = true;

            // check whether tello poses and their targets are available
            // if not available, emergency flag is to be true
            if (utility_->getTelloPoses(telloPoses) && utility_->getTargets(targets)) {
                emergency = false;
                Eigen::VectorXd output = getOutput(telloPoses, targets);
                Eigen::VectorXd controlInput = lqr_.getControlInput(output);
                // CAUTION: calculated control input must be checked before publishing
                // if invalid, emergency flag must be set to true
                if (checkNaN(controlInput)) {
                    emergency = true;
                } else {
                    // back stepping?
                    controlInput = modifyControlInputAccordingToYaw(controlInput, telloPoses);
                    // check max or min
                    controlInput = clipVelocities(controlInput);
                    // send commands
                    for (int i = 0; i < telloNum_; ++i) {
                        printf("%dth: %lf %lf %lf %lf\n", i, controlInput(4 * i + 0), controlInput(4 * i + 1), controlInput(4 * i + 2), controlInput(4 * i + 3));
                        utility_->publishCmd(controlInput(4 * i + 0), controlInput(4 * i + 1),
                            controlInput(4 * i + 2), controlInput(4 * i + 3), i);
                    }
                    printf("\n");
                }
            } else {
                ROS_ERROR("tello poses or target poses are not available");
            }

            // send zero velocities to all the tellos if emergency things occur.
            if (emergency) {
                for (int i = 0; i < telloNum_; ++i)
                    utility_->publishCmd(0.0, 0.0, 0.0, 0.0, i);
                ROS_ERROR("stop all the tellos.");
            }
            loopRate.sleep();
        }
    }
}; // class FormationFlightLQR

int main(int argc, char **argv) {
    // sleep(5);
    ros::init(argc, argv, "formation_flight_lqr");
    FormationFlightLQR node;
    sleep(1);
    node.spin();
    return 0;
}
