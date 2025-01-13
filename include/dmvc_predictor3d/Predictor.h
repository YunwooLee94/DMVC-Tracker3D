//
// Created by larr-planning on 25. 1. 13.
//

#ifndef DMVC_TRACKER3D_PREDICTOR_H
#define DMVC_TRACKER3D_PREDICTOR_H
#include <ros/ros.h>
#include <dmvc_predictor3d/PredictionVisualizer.h>
#include <dmvc_utils3d/eigenmvn.h>
#include <dmvc_tracker3d/ObjectState.h>
#include <dmvc_tracker3d/ObjectStateList.h>
#include <dmvc_tracker3d/PolyState.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <decomp_util/seed_decomp.h>
#include <decomp_ros_utils/data_ros_utils.h>

namespace dmvc3d{
    struct PredictonParam{
        bool is_unstructured;
        int num_thread;
        int num_sample;
        double horizon;
        double object_size;
        double acc_max_target;
    };
    class Predictor{
    private:
        ros::NodeHandle nh_;
        PredictonParam param_;

        dmvc3d::VisualizationParam vis_param_;
        dmvc3d::PredictionVisualizer visualizer_;

        ros::Subscriber target_state_subscriber_;
        ros::Subscriber obstacle_state_list_subscriber_;
        ros::Subscriber pcl_subscriber_;
        void TargetStateCallback(const dmvc_tracker3d::ObjectState &msg);
        void ObjectStateListCallback(const dmvc_tracker3d::ObjectStateList &msg);
        void PclCallback(const sensor_msgs::PointCloud2::ConstPtr &pcl_msgs);

        ros::Publisher corridor_publisher_;
        ros::Publisher raw_primitive_publisher_;
        ros::Publisher feasible_primitive_publisher_;
        ros::Publisher best_primitive_publisher_;
        ros::Publisher prediction_result_publisher_;

        dmvc3d::State current_target_state_;
        vector<dmvc3d::PrimitiveTarget> current_obstacle_primitive_list_;
        vec_Vec3f point_cloud_3d_;
        vec_E<Polyhedron3D> polys_;

        vector<dmvc3d::Point> end_points_;
        vector<dmvc3d::PrimitiveTarget> primitive_;
        vector<dmvc3d::uint> safe_index_;
        uint best_prediction_index_;

        bool Predict();
        void SampleEndPoints();
        void SampleEndPointsSubProcess(const int &start_idx, const int &end_idx, vector<Point> &endpoint_list_sub);
        void GeneratePrimitives();
        void GeneratePrimitivesSubProcess(const int &start_idx, const int &end_idx, vector<PrimitiveTarget> &primitive_sub);
        void GetSafeIndex();
        void GetSafeIndexSubProcess(const int &start_idx, const int &end_idx, vector<uint> &safe_index_sub);
        void GetSafeIndexUnstructuredSubProcess(const LinearConstraint3D &constraint, const int &start_idx, const int &end_idx, std::vector<uint> &safe_index_sub);

        LinearConstraint3D GenerateCorridor();
        void GetBestIndex();
        void GetBestIndexSubProcess(const int &start_idx, const int &end_idx, std::pair<uint, double> &best_index_sub);
        void UpdateResult();
        void EraseResult();

    public:
        Predictor();
        void Run();
    };
}
#endif //DMVC_TRACKER3D_PREDICTOR_H
