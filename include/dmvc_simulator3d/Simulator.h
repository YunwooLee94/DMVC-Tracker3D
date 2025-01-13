//
// Created by larr-laptop on 25. 1. 12.
//

#ifndef DMVC_TRACKER3D_SIMULATOR_H
#define DMVC_TRACKER3D_SIMULATOR_H
#include <ros/ros.h>
#include <string>
#include <fstream>
#include <istream>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <dmvc_tracker3d/ControlInput.h>
#include <dmvc_tracker3d/ControlInputList.h>
#include <dmvc_tracker3d/ObjectState.h>
#include <dmvc_tracker3d/ObjectStateList.h>
#include <dmvc_utils3d/Utils.h>
#include <algorithm>
#include <vector>
#include <ctime>

namespace dmvc3d{
    class Simulator{
    private:
        ros::NodeHandle nh_;
        bool is_unstructured_{false};
        double inflation_size_{0.5};
        double point_resolution_{0.01};
        double simulation_dt_;
        double simulation_frequency_;
        string map_frame_id_;
        double agent_size_;
        vector<int> object_idx_list_;
        int total_object_number_in_file_;
        int object_number_;
        int target_idx_;
        int tracker_number_;
//        int repetition_; (ANALYSIS)
        int moving_obstacle_number_;
//        int total_test_number_; (ANALYSIS)
        vector<int> obstacle_idx_list_;

        pcl::PointCloud<pcl::PointXYZ> point_cloud_;
        pcl::PointCloud<pcl::PointXYZ> point_cloud_range_;

        vector<StateHistory> object_history_list_;
        vector<State> current_tracker_state_;
        vector<ControlInput> tracker_control_input;
        State current_target_state_;
        vector<State> current_obstacle_state_list_;

        string initial_state_file_name_;
        string object_history_file_name_;
        string obstacle_configuration_file_name_;

        void ReadInitialTrackerStateList();
        void ReadObjectTrajectory();
        void ReadObstacleConfiguration();
        void ShuffleScenario();
        void UpdateDynamics(const double &t);
        void PrepareRosMsgs(const double &t);
        void PublishRosMsgs();
        void ProcessPointCloud();
//        std::vector<int> generateUniqueRandomArray(int size, int lower_bound, int upper_bound);

        visualization_msgs::MarkerArray obstacle_list_vis_;
        visualization_msgs::Marker obstacle_vis_;
        visualization_msgs::Marker target_vis_;
        visualization_msgs::MarkerArray tracker_list_vis_;
        visualization_msgs::Marker tracker_vis_;
        visualization_msgs::MarkerArray pcl_boxes_vis_;

        ros::Publisher target_vis_publisher_;
        ros::Publisher obstacle_list_vis_publisher_;
        ros::Publisher tracker_list_vis_publisher_;
        ros::Publisher pcl_publisher_;
        ros::Publisher pcl_boxes_vis_publisher_;

        ros::Subscriber control_input_subscriber_;
        void control_input_callback(const dmvc_tracker3d::ControlInputList &msg);
        ros::Publisher target_state_publisher_;
        ros::Publisher obstacle_state_list_publisher_;
        ros::Publisher tracker_state_list_publisher_;
        dmvc_tracker3d::ObjectState target_state_msg_;
        dmvc_tracker3d::ObjectStateList obstacle_state_list_msg_;
        dmvc_tracker3d::ObjectStateList tracker_state_list_msg_;

    public:
        Simulator();
        void Run();
    };
}


#endif //DMVC_TRACKER3D_SIMULATOR_H
