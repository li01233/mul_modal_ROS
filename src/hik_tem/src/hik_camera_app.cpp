#include "hik_camera.h"
#include <stdio.h>
#include <ros/ros.h>


int main(int argc, char *argv[]){
    ros::init(argc,argv,"hik_camera_app");

    HikCamera camera;
    camera.run();
    return 0;
}