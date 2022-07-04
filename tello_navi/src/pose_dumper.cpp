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
#include <tello_navi/FormationFlightUtility.h>

class PoseDumper {
private:
    ros::NodeHandle nh_;
    FormationFlightUtility *utility_;
    int telloNum_;
    double dumpHz_;
    std::string filePath_;

public:
    PoseDumper(void):
        nh_("~"),
        dumpHz_(20.0),
        telloNum_(6),
        filePath_("/tmp/tello_poses.txt")
    {
        nh_.param("tello_num", telloNum_, telloNum_);
        nh_.param("dump_hz", dumpHz_, dumpHz_);
        nh_.param("file_path", filePath_, filePath_);

        utility_ = new FormationFlightUtility(nh_, telloNum_);
    }

    void spin(void) {
        FILE *fp = fopen(filePath_.c_str(), "w");
        if (fp == NULL) {
            ROS_ERROR("Cannot open a file %s", filePath_.c_str());
            exit(1);
        }

        ros::Rate loopRate(dumpHz_);
        while (ros::ok()) {
            ros::spinOnce();
            std::vector<Pose> telloPoses;
            if (utility_->getTelloPoses(telloPoses)) {
                for (int i = 0; i < telloNum_; ++i) {
                    double x = telloPoses[i].getX();
                    double y = telloPoses[i].getY();
                    double z = telloPoses[i].getZ();
                    double roll = telloPoses[i].getRoll();
                    double pitch = telloPoses[i].getPitch();
                    double yaw = telloPoses[i].getYaw();
                    if (i < telloNum_ - 1) {
                        fprintf(fp, "%lf %lf %lf %lf %lf %lf ", x, y, z, roll, pitch, yaw);
                        printf("%lf %lf %lf %lf %lf %lf ", x, y, z, roll, pitch, yaw);
                    } else {
                        fprintf(fp, "%lf %lf %lf %lf %lf %lf\n", x, y, z, roll, pitch, yaw);
                        printf("%lf %lf %lf %lf %lf %lf\n", x, y, z, roll, pitch, yaw);
                    }
                }
            }
            loopRate.sleep();
        }

        fclose(fp);
    }

}; // class PoseDumper

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_dumper");
    PoseDumper node;
    node.spin();
    return 0;
}