//
// Created by larr-planning on 25. 1. 13.
//

#ifndef DMVC_TRACKER3D_PREDICTIONVISUALIZER_H
#define DMVC_TRACKER3D_PREDICTIONVISUALIZER_H
#include <visualization_msgs/MarkerArray.h>
#include <dmvc_utils3d/bernstein_util.h>
namespace dmvc3d{
    struct VisualizationParam{
        std::string frame_id;
        struct{
            bool publish{false};
            int num_time_sample{10};
            double proportion{0.0};
            double line_scale{0.01};
            double color_a{0.0};
            double color_r{0.0};
            double color_g{0.0};
            double color_b{0.0};
        }raw_primitives;
        struct{
            bool publish{false};
            int num_time_sample{10};
            double proportion{0.0};
            double line_scale{0.01};
            double color_a{0.0};
            double color_r{0.0};
            double color_g{0.0};
            double color_b{0.0};
        }feasible_primitives;
        struct{
            int num_time_sample{10};
            double line_scale{0.01};
            double color_a{0.0};
            double color_r{0.0};
            double color_g{0.0};
            double color_b{0.0};
        }best_primitive;
    };
    class PredictionVisualizer{
    public:
        PredictionVisualizer();
        void UpdateParameter(const dmvc3d::VisualizationParam &param);;


        visualization_msgs::MarkerArray VisualizeRawPrimitives(const vector<dmvc3d::PrimitiveTarget> & primitive);
        visualization_msgs::MarkerArray VisualizeFeasiblePrimitives(const vector<dmvc3d::PrimitiveTarget> &primitive, const vector<dmvc3d::uint> &feasible_index);
        visualization_msgs::Marker VisualizeBestPrimitive(const vector<dmvc3d::PrimitiveTarget>&primitive, const uint& best_index);
    private:
        visualization_msgs::Marker raw_primitive_;
        visualization_msgs::Marker feasible_primitive_;
        visualization_msgs::Marker best_primitive_;
        VisualizationParam param_;
    };
}

#endif //DMVC_TRACKER3D_PREDICTIONVISUALIZER_H
