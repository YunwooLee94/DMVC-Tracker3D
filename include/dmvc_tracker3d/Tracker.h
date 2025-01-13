//
// Created by larr-planning on 25. 1. 13.
//

#ifndef DMVC_TRACKER3D_TRACKER_H
#define DMVC_TRACKER3D_TRACKER_H
#include <dmvc_tracker3d/PlannerBase.h>
#include <dmvc_utils3d/eigenmvn.h>

namespace dmvc3d{
    class Tracker{
    public:
        Tracker(const TrackingParam & param, shared_ptr<PlannerBase> p_base);

        shared_ptr <PlannerBase> p_base_;
        bool Plan(const double &t_trigger);
        void UpdateResultToBase(const bool& is_success);
    private:
        TrackingParam param_;
        vector<vector<dmvc3d::PrimitivePlanning>> primitive_;
        vector<dmvc3d::PrimitivePlanning> obstacle_primitive_list_;
        dmvc3d::PrimitivePlanning target_trajectory_;
        vector<dmvc3d::State> current_tracker_state_;
        int num_tracker_;
        int num_obstacle_;

        vector<vector<dmvc3d::Point>> end_points_;
        vector<vector<dmvc3d::uint>> safe_index_;
        vector<vector<dmvc3d::uint>> inter_safe_index_;
        vector<vector<dmvc3d::uint>> inter_visible_index_;
        vector<vector<dmvc3d::uint>> dynamically_feasible_index_;

        vector<dmvc3d::uint> best_index_;
        vector<vector<AffineCoeff2D>> moving_buffered_voroni_cell_;
        vector<vector<AffineCoeff2D>> visibility_cell_;

        vec_Vec3f point_cloud_3d_;
        vec_E<Polyhedron3D> polys_;
        vector<LinearConstraint3D> corridor_constraints_;

        void UpdateValue(const double &t);
        bool CheckInfoAvailable();

        void SampleEndPoint();
        void SampleEndPointThread(const int &tracker_idx, const int & start_idx, const int & end_idx, vector<dmvc3d::Point>& endpoint_list_sub);
        void GeneratePrimitive(const double &t);
        void GeneratePrimitiveThread(const int &tracker_idx, const double &t, const int &start_idx, const int &end_idx, vector<dmvc3d::PrimitivePlanning> & primitive_list_sub);
        void GetSafeIndex();
        void GetSafeIndexThread(const int &tracker_idx, const int &start_idx, const int &end_idx, vector<dmvc3d::uint> &safe_idx_sub);
        void GenerateCorridor();
        void GetSafeIndexUnstructuredThread(const int &tracker_idx, const int &start_idx, const int &end_idx, vector<dmvc3d::uint> &safe_idx_sub);
        void GetInterSafetyIndex();
        void GetInterSafetyIndexThread(const int &tracker_idx, const int &start_idx, const int &end_idx, vector<dmvc3d::uint> &inter_safe_idx_sub);
        void GetDynamicallyFeasibleIndex();
        void GetDynamicallyFeasibleIndexThread(const int &tracker_idx, const int &start_idx, const int &end_idx, vector<dmvc3d::uint> &dyn_feas_idx_sub);
        void GetInterVisibleIndex();
        void GetInterVisibleIndexThread(const int &tracker_idx, const int &start_idx, const int &end_idx, vector<dmvc3d::uint> &inter_visible_idx_sub);

        void GetInterSafetyNonCooperativeIndex();
        void GetInterSafetyNonCooperativeIndexThread(const int &tracker_idx, const int &start_idx, const int &end_idx, vector<dmvc3d::uint> &inter_safe_idx_sub);

        void GetBestIndex();
        void GetBestIndexThread(const int &tracker_idx, const int &start_idx, const int &end_idx, std::pair<uint,double> &score_pair);
        void CalculateMovingBufferedVoronoiCell();
        void CalculateVisibilityCell();
        vector<AffineCoeff2D> CalculateSingleVisibilityCell(const dmvc3d::State &tracker_pos, const dmvc3d::State &neighbor_pos);
    };
}

#endif //DMVC_TRACKER3D_TRACKER_H
