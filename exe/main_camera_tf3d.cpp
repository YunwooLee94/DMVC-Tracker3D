//
// Created by larr-planning on 25. 1. 17.
//
#include <dmvc_camera_tf3d/CameraTfPublisher.h>

int main(int argc, char **argv){
    ros::init(argc,argv,"dmvc_camera_tf3d");
    dmvc3d::CameraTfPublisher camera;
    camera.Run();
    return 0;
}