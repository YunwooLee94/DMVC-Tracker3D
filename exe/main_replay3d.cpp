//
// Created by larr-planning on 25. 1. 15.
//
#include <dmvc_replay3d/Replayer.h>

int main(int argc, char **argv){
    ros::init(argc,argv,"dmvc_replayer3d");
    dmvc3d::Replayer replayer;
    replayer.Run();
    return 0;
}
