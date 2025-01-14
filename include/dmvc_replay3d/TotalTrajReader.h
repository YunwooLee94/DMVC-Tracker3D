//
// Created by larr-planning on 25. 1. 14.
//

#ifndef DMVC_TRACKER3D_TOTALTRAJREADER_H
#define DMVC_TRACKER3D_TOTALTRAJREADER_H

#include <ros/ros.h>
#include <istream>
#include <fstream>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <string>

namespace dmvc3d{
    class TotalTrajReader{
    private:
        ros::NodeHandle nh_;
        int num_obstacle_;
        int num_tracker_;
        int num_target_;
        std::string file_name_;
        std::string baseline_file_name_;
        std::string obstacle_configuration_file_name_;
        double inflation_size_;
        std::string map_frame_id_;
        ros::Publisher pcl_boxes_vis_publisher_;
        visualization_msgs::MarkerArray pcl_boxes_vis_;

        visualization_msgs::MarkerArray target_path_vis_;
        visualization_msgs::MarkerArray tracker_path_list_vis_;
        visualization_msgs::MarkerArray obstacle_path_list_vis_;
        visualization_msgs::Marker bearing_list_vis_;

        ros::Publisher target_path_publisher_;
        ros::Publisher tracker_path_publisher_;
        ros::Publisher obstacle_path_publisher_;
        ros::Publisher bearing_history_publisher_;

        visualization_msgs::MarkerArray target_path_baseline_vis_;
        visualization_msgs::MarkerArray tracker_path_list_baseline_vis_;
        visualization_msgs::Marker bearing_list_baseline_vis_;

        ros::Publisher target_path_baseline_publisher_;
        ros::Publisher tracker_path_baseline_publisher_;
        ros::Publisher bearing_history_baseline_publisher_;


        void ReadTotalTrajectory();
        void ReadBaselineTotalTrajectory();
        void Publish();
        void UnstructuredObstacleConfiguration();
    public:
        TotalTrajReader();
        void Run();
    };
}


#endif //DMVC_TRACKER3D_TOTALTRAJREADER_H
