//
// Created by larr-planning on 25. 1. 13.
//

#ifndef DMVC_TRACKER3D_PLANNERBASE_H
#define DMVC_TRACKER3D_PLANNERBASE_H
#include <mutex>
#include <string>
#include <dmvc_utils3d/Utils.h>
#include <dmvc_utils3d/bernstein_util.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_ros_utils/data_ros_utils.h>

namespace dmvc3d{
    struct TrackingParam{
        double horizon{0.0};
        bool is_unstructured{false};
        int num_thread{1};
        int num_sample_planning{1};
        double r_min{0.0};
        double r_max{0.0};
        double elevation_range{0.0};
        double safe_distance{0.0};
        double vel_max{0.0};
        double acc_max{0.0};
        double object_radius{0.0};
        double terminal_weight{0.0};
        double distance_max{10.0};
        bool is_experiment{false};
        struct{
            double min_x{0.0};
            double min_y{0.0};
            double max_x{0.0};
            double max_y{0.0};
        }axis_limit;
    };

    class PlannerBase{
    private:
    public:
        std::mutex mutex_set_[2];
        // BUFFER DATA FROM ROS WRAPPER
        vector<dmvc3d::State> current_tracker_list_read_;
        vector<dmvc3d::State> current_obstacle_list_read_;
        dmvc3d::PrimitivePlanning target_prediction_read_;

        vector<vector<dmvc3d::PrimitivePlanning>> tracker_raw_primitives_;
        vector<vector<dmvc3d::uint>> tracker_feasible_index_;
        vector<dmvc3d::uint> tracker_best_index_;

        vector<vector<AffineCoeff2D>> buffered_voronoi_cell_;
        vector<vector<AffineCoeff2D>> visibility_cell_;

        vec_Vec3f point_cloud_3d_;
        vec_E<Polyhedron3D> polys_;

        bool is_tracker_info_{false};
        bool is_target_info_{false};
        bool is_obstacle_info_{false};

        bool success_flag_{false};

        void SetTrackerPrimitives(const vector<vector<PrimitivePlanning>>&tracker_primitive);
        void EraseTrackerPrimitives();

        void SetFeasibleIndex(const vector<vector<dmvc3d::uint>> &feasible_index);
        void EraseFeasibleIndex();

        void SetBestIndex(const vector<dmvc3d::uint> &best_index);
        void EraseBestIndex();

        void SetMovingBufferedVoronoiCell(const vector<vector<AffineCoeff2D>> &voronoi_cell);
        void EraseMovingBufferedVoronoiCell();

        void SetVisibilityCell(const vector<vector<AffineCoeff2D>> &visibility_cell);
        void EraseVisibilityCell();

        void SetCorridorVis(const vec_E<Polyhedron3D> &poly);
        void EraseCorridorVis();

    };
}
#endif //DMVC_TRACKER3D_PLANNERBASE_H