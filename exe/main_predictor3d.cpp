//
// Created by larr-planning on 25. 1. 13.
//
#include <dmvc_predictor3d/Predictor.h>

int main(int argc, char**argv){
    ros::init(argc,argv,"dmvc_predictor3d");
    dmvc3d::Predictor predictor;
    predictor.Run();
    return 0;
}