//
// Created by larr-planning on 25. 1. 17.
//

#ifndef DMVC_TRACKER3D_CAMERATFPUBLISHER_H
#define DMVC_TRACKER3D_CAMERATFPUBLISHER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <dmvc_tracker3d/ObjectStateList.h>
#include <dmvc_tracker3d/ObjectState.h>
#include <sensor_msgs/Illuminance.h>

namespace dmvc3d{
    class CameraTfPublisher{
        struct ObjectState{
            double x;
            double y;
            double z;
            double yaw;
        };
    public:
        void Run();
        CameraTfPublisher();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber target_state_subscriber_;
        void TargetStateCallback(const dmvc_tracker3d::ObjectState &msg);
        ros::Subscriber tracker_state_list_subscriber_;
        void TrackerStateListCallback(const dmvc_tracker3d::ObjectStateList &msg);
        std::vector<ObjectState> tracker_state_list_;
        ObjectState target_state_;
        bool is_received_target_state_{false};
        bool is_received_tracker_state_list_{false};
        void PublishCameraTf();
        ros::Publisher illuminate_publisher1_;
        ros::Publisher illuminate_publisher2_;
        ros::Publisher illuminate_publisher3_;
        ros::Publisher illuminate_publisher0_;
    };
}

#endif //DMVC_TRACKER3D_CAMERATFPUBLISHER_H
