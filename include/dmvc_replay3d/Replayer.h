//
// Created by larr-planning on 25. 1. 15.
//

#ifndef DMVC_TRACKER3D_REPLAYER_H
#define DMVC_TRACKER3D_REPLAYER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <dmvc_tracker3d/ObjectState.h>
#include <dmvc_tracker3d/ObjectStateList.h>
#include <vector>
#include <fstream>

namespace dmvc3d{
    struct ReplayerParam{
        std::string map_frame_id;
    };
    struct State{
        double px;
        double py;
        double pz;
        double vx;
        double vy;
        double vz;
        double yaw;
    };
    class Replayer{
    private:
        ros::NodeHandle nh_;
        ReplayerParam param_;

        bool tracker_info_flag_{false};
        bool target_info_flag_{false};
        bool dynamic_obstacle_info_flag{false};
        bool is_dynamic_experiment_{false};
        bool do_logging_{false};
        double t0_;
        double GetCurrentTime();

        std::string state_history_file_name_;

        std::vector<State> tracker_list_;
        State target_;
        std::vector<State> dynamic_obstacle_list_;

        void PublishMsgs();
        void WriteStateHistory();

        // Subscriber
        ros::Subscriber tracker_list_subscriber_;
        ros::Subscriber target_subscriber_;
        ros::Subscriber dynamic_obstacle_subscriber_;
//        ros::Subscriber<dmvc_tracker::ObjectStateList>("/dmvc_simulator/tracker_state_list",1,&Replayer::)
        // Subscriber callback
        void TrackerListCallback(const dmvc_tracker3d::ObjectStateList& tracker_list);
        void TargetCallback(const dmvc_tracker3d::ObjectState& target);
        void DynamicObstacleCallback(const dmvc_tracker3d::ObjectStateList & obstacle_list);
        // Publisher
        ros::Publisher tracker_vis_publisher_;
        ros::Publisher target_vis_publisher_;
        ros::Publisher dynamic_obstacle_vis_publisher_;
        ros::Publisher bearing_vector_vis_publisher_;
    public:
        void Run();
        Replayer();
    };
}

#endif //DMVC_TRACKER3D_REPLAYER_H
