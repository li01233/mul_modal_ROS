#include "IMU.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu");

    Imu imu;
    imu.run();

    return 0;
}