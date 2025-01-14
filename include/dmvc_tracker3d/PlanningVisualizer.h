//
// Created by larr-planning on 25. 1. 13.
//

#ifndef DMVC_TRACKER3D_PLANNINGVISUALIZER_H
#define DMVC_TRACKER3D_PLANNINGVISUALIZER_H

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <dmvc_utils3d/Utils.h>
#include <dmvc_utils3d/bernstein_util.h>
#include <dmvc_utils3d/geo_utils.hpp>
#include "std_msgs/ColorRGBA.h"
#include "../dmvc_utils3d/convhull_3d.h"
typedef std_msgs::ColorRGBA ColorMsg;
typedef std::vector<ColorMsg> ColorMsgs;
typedef geometry_msgs::Point PointMsg;
typedef Eigen::Vector3d Vec3;
typedef std::vector<Eigen::Vector3d> Vec3List;
#define MOVING_BUFFERED_VORONOI_CELL 1
#define MOVING_VISIBILITY_CELL 2


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
        struct{
            struct{
                double x;
                double y;
                double z;
            }axis_min;
            struct{
                double x;
                double y;
                double z;
            }axis_max;
            struct{
                double color_a{0.0};
                double color_r{0.0};
                double color_g{0.0};
                double color_b{0.0};
            }visibility;
            struct{
                double color_a{0.0};
                double color_r{0.0};
                double color_g{0.0};
                double color_b{0.0};
            }voronoi;
        }cell;
    };
    class PlanningVisualizer{
    public:
        PlanningVisualizer(const dmvc3d::VisualizationParam & vis_param);
        void UpdateParameter(const dmvc3d::VisualizationParam & param);

        visualization_msgs::MarkerArray VisualizeRawPrimitives(const vector<vector<dmvc3d::PrimitivePlanning>> & primitive);
        visualization_msgs::MarkerArray VisualizeFeasiblePrimitives(const vector<vector<dmvc3d::PrimitivePlanning>> &primitive, const vector<vector<dmvc3d::uint>> &feasible_index);
        visualization_msgs::MarkerArray VisualizeBestPrimitive(const vector<vector<dmvc3d::PrimitivePlanning>>&primitive, const vector<uint>& best_index);
        visualization_msgs::MarkerArray VisualizeBufferedVoronoiCell(const vector<vector<AffineCoeff2D>> &buffered_voronoi_cell);
        visualization_msgs::MarkerArray VisualizeVisibilityCell(const vector<vector<AffineCoeff2D>> & visibility_cell);

    private:
        visualization_msgs::Marker raw_primitive_;
        visualization_msgs::Marker feasible_primitive_;
        visualization_msgs::Marker best_primitive_;
        visualization_msgs::Marker single_voronoi_cell_;
        visualization_msgs::Marker single_visibility_cell_;
        ColorMsg voronoi_cell_color_;
        ColorMsg visibility_cell_color_;
        VisualizationParam param_;

        inline PointMsg GetDefaultPointMsg(){
            PointMsg p;
            p.x = 0.0, p.y = 0.0, p.z = 0.0;
            return p;
        }
        inline geometry_msgs::Quaternion GetDefaultQuaternionMsg(){
            geometry_msgs::Quaternion q;
            q.w = 1.0, q.x = 0.0, q.y = 0.0, q.z =0.0;
            return q;
        }
        inline geometry_msgs::Vector3  GetDefaultScaleMsg(){
            geometry_msgs::Vector3 v;
            v.x = 1.0, v.y = 1.0, v.z = 1.0;
            return v;
        }
        visualization_msgs::Marker VisualizeConvexHull(const Vec3List &convex_hull,const int &tracker_idx, const uint &cell_type);
        vector<AffineCoeff3D> GetHalfSpaceFromBoundary();   // [xmin,ymin,zmin] ~[xmax,ymax,zmax]
        Vec3List GetFeasibleVertices(const vector<AffineCoeff2D> &constraint);
    };
}

#endif //DMVC_TRACKER3D_PLANNINGVISUALIZER_H
