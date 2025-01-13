//
// Created by larr-laptop on 25. 1. 12.
//
#include <dmvc_simulator3d/Simulator.h>

int main(int argc, char**argv){
    ros::init(argc,argv,"dmvc_simulator");
    dmvc3d::Simulator simulator;
    simulator.Run();
    return 0;
}