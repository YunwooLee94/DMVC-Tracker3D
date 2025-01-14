//
// Created by larr-planning on 25. 1. 13.
//
#include <dmvc_tracker3d/Wrapper.h>
int main(int argc, char**argv){
    ros::init(argc,argv,"dmvc_tracker3d");
    dmvc3d::Wrapper wrapper;
    wrapper.Run();
    return 0;
}