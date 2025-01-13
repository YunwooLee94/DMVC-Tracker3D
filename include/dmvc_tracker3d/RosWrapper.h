//
// Created by larr-planning on 25. 1. 13.
//

#ifndef DMVC_TRACKER3D_ROSWRAPPER_H
#define DMVC_TRACKER3D_ROSWRAPPER_H
#include <ros/ros.h>
#include <thread>
#include <dmvc_tracker3d/Tracker.h>
#include <dmvc_tracker3d/PlanningVisualizer.h>
#include <dmvc_tracker3d/ObjectState.h>
#include <dmvc_tracker3d/ObjectStateList.h>
#include <dmvc_tracker3d/PolyState.h>
#include <dmvc_tracker3d/ControlInput.h>
#include <dmvc_tracker3d/ControlInputList.h>

namespace dmvc3d{
    struct RosParam{
        double control_frequency;
        double planning_frequency;
    };
    class RosWrapper{
    public:
        RosWrapper(shared_ptr<PlannerBase> p_base);
        void RunRos();
        double GetCurrentTime(){return ros::Time::now().toSec()-t0_;};
        dmvc3d::TrackingParam GetTrackingParam();
        dmvc3d::VisualizationParam GetVisualizationParam();
        double GetPlanningFrequency(){return ros_param_.planning_frequency;};
        double GetControlFrequency(){return ros_param_.control_frequency;};
        void InitSubscriberAndPublisher();
    private:
        ros::NodeHandle nh_;
        shared_ptr<PlannerBase> p_base_;
        dmvc3d::VisualizationParam vis_param_;
        dmvc3d::RosParam ros_param_;
        dmvc3d::TrackingParam planning_param_;
        dmvc3d::PlanningVisualizer * visualizer_;
        double t0_;
        void PrepareRosMsgs();
        void PublishRosMsgs();

        ros::Subscriber tracker_list_state_subscriber_;
        ros::Subscriber target_prediction_subscriber_;
        ros::Subscriber obstacle_state_list_subscriber_;

        ros::Publisher corridor_publisher_;
        ros::Subscriber pcl_subscriber_;
        void PclCallback(const sensor_msgs::PointCloud2::ConstPtr &pcl_msgs);

        ros::Publisher tracker_raw_primitives_publisher_;
        ros::Publisher tracker_feasible_primitives_publisher_;
        ros::Publisher tracker_best_trajectory_publisher_;

        ros::Publisher buffered_voronoi_cell_publisher_;
        ros::Publisher visibility_cell_publisher_;

        ros::Publisher tracker_control_input_publisher_;


        void ObstacleStateListCallback(const dmvc_tracker3d::ObjectStateList &msg);
        void TrackerStateListCallback(const dmvc_tracker3d::ObjectStateList &msg);
        void TargetPredictionCallback(const dmvc_tracker3d::PolyState &msg);

        dmvc_tracker3d::ControlInputList GenerateControlInput(const vector<vector<dmvc3d::PrimitivePlanning>> & primitive, const vector<dmvc3d::uint> &best_index, const double &time_t);

    };
}


#endif //DMVC_TRACKER3D_ROSWRAPPER_H
