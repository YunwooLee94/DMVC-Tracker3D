//
// Created by larr-planning on 25. 1. 14.
//
#include <dmvc_replay3d/TotalTrajReader.h>
int main(int argc, char**argv){
    ros::init(argc,argv,"dmvc_total3d");
    dmvc3d::TotalTrajReader traj_reader;
    traj_reader.Run();
    return 0;
}