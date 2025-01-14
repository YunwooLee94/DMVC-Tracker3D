//
// Created by larr-planning on 25. 1. 13.
//

#ifndef DMVC_TRACKER3D_WRAPPER_H
#define DMVC_TRACKER3D_WRAPPER_H

#include <dmvc_tracker3d/RosWrapper.h>
namespace dmvc3d{
    using namespace std;
    class Wrapper{
    public:
        Wrapper();
        void Run();
    private:
        ros::NodeHandle nh_;
        thread thread_planner_;
        thread thread_ros_wrapper_;
        shared_ptr<PlannerBase> p_base_shared_;
        dmvc3d::RosWrapper* ros_wrapper_ptr_;
        dmvc3d::Tracker * tracker_;

//        void InitRosSubscriberAndPublisher();
//        void UpdateTrackerParameters();
        void RunPlanning();
    };
}

#endif //DMVC_TRACKER3D_WRAPPER_H
