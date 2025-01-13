//
// Created by larr-planning on 25. 1. 13.
//
#define CONVHULL_3D_ENABLE

#include <dmvc_tracker3d/PlanningVisualizer.h>

visualization_msgs::MarkerArray
dmvc3d::PlanningVisualizer::VisualizeRawPrimitives(const vector<vector<dmvc3d::PrimitivePlanning>> &primitive) {
    visualization_msgs::MarkerArray visual_output;
    if (not param_.raw_primitives.publish)
        return visual_output;
    if (primitive.empty())
        return visual_output;
    for (int tracker_idx = 0; tracker_idx < primitive.size(); tracker_idx++) {
        raw_primitive_.ns = "TRACKER_RAW_" + std::to_string(tracker_idx);
        std::vector<double> time_seq;
        double seg_t0 = primitive[tracker_idx][0].t0;
        double seg_tf = primitive[tracker_idx][0].tf;
        for (int i = 0; i < param_.raw_primitives.num_time_sample; i++)
            time_seq.push_back(seg_t0 + (double) i * (seg_tf - seg_t0) /
                                        (double) (param_.raw_primitives.num_time_sample - 1));
        geometry_msgs::Point temp_point;
        int num_primitive_vis = (int) ((double) primitive[tracker_idx].size() * param_.raw_primitives.proportion);
        int sample_step = primitive[tracker_idx].size() / num_primitive_vis;
        double ctrl_pts_x[4];
        double ctrl_pts_y[4];
        double ctrl_pts_z[4];
        for (int idx = 0; idx < num_primitive_vis - 1; idx++) {
            raw_primitive_.points.clear();
            raw_primitive_.id = idx;
            for (int i = 0; i < 4; i++) {
                ctrl_pts_x[i] = primitive[tracker_idx][sample_step * idx].ctrl_x[i];
                ctrl_pts_y[i] = primitive[tracker_idx][sample_step * idx].ctrl_y[i];
                ctrl_pts_z[i] = primitive[tracker_idx][sample_step * idx].ctrl_z[i];
            }
            for (int i = 0; i < time_seq.size(); i++) {
                temp_point.x =
                        getBernsteinValue(ctrl_pts_x, time_seq[i], primitive[tracker_idx][sample_step * idx].t0,
                                          primitive[tracker_idx][sample_step * idx].tf, 3);
                temp_point.y =
                        getBernsteinValue(ctrl_pts_y, time_seq[i], primitive[tracker_idx][sample_step * idx].t0,
                                          primitive[tracker_idx][sample_step * idx].tf, 3);
                temp_point.z =
                        getBernsteinValue(ctrl_pts_z, time_seq[i], primitive[tracker_idx][sample_step * idx].t0,
                                          primitive[tracker_idx][sample_step * idx].tf, 3);
                raw_primitive_.points.push_back(temp_point);
            }
            visual_output.markers.push_back(raw_primitive_);
        }
    }

    return visual_output;
}

visualization_msgs::MarkerArray
dmvc3d::PlanningVisualizer::VisualizeFeasiblePrimitives(const vector<vector<dmvc3d::PrimitivePlanning>> &primitive,
                                                        const vector<vector<dmvc3d::uint>> &feasible_index) {
    visualization_msgs::MarkerArray visual_output;
    if (not param_.feasible_primitives.publish)
        return visual_output;
    if (primitive.empty())
        return visual_output;
    if (feasible_index.empty())
        return visual_output;
    for (int tracker_idx = 0; tracker_idx < primitive.size(); tracker_idx++) {
        feasible_primitive_.ns = "TRACKER_FEASIBLE_" + std::to_string(tracker_idx);
        std::vector<double> time_seq;
        double seg_t0 = primitive[tracker_idx][0].t0;
        double seg_tf = primitive[tracker_idx][0].tf;
        for (int i = 0; i < param_.feasible_primitives.num_time_sample; i++)
            time_seq.push_back(seg_t0 + (double) i * (seg_tf - seg_t0) /
                                        (double) (param_.feasible_primitives.num_time_sample - 1));
        geometry_msgs::Point temp_point;
        int num_primitive_vis =
                (int) ((double) feasible_index[tracker_idx].size() * param_.feasible_primitives.proportion);
        int sample_step;
        if (num_primitive_vis != 0)
            sample_step = feasible_index[tracker_idx].size() / num_primitive_vis;
        else
            num_primitive_vis = 1;

        double ctrl_pts_x[4];
        double ctrl_pts_y[4];
        double ctrl_pts_z[4];
        for (int idx = 0; idx < num_primitive_vis - 1; idx++) {
            feasible_primitive_.points.clear();
            feasible_primitive_.id = idx;
            for (int i = 0; i < 4; i++) {
                ctrl_pts_x[i] = primitive[tracker_idx][feasible_index[tracker_idx][sample_step * idx]].ctrl_x[i];
                ctrl_pts_y[i] = primitive[tracker_idx][feasible_index[tracker_idx][sample_step * idx]].ctrl_y[i];
                ctrl_pts_z[i] = primitive[tracker_idx][feasible_index[tracker_idx][sample_step * idx]].ctrl_z[i];
            }
            for (int i = 0; i < time_seq.size(); i++) {
                temp_point.x = getBernsteinValue(
                        ctrl_pts_x, time_seq[i],
                        primitive[tracker_idx][feasible_index[tracker_idx][sample_step * idx]].t0,
                        primitive[tracker_idx][sample_step * idx].tf, 3);
                temp_point.y = getBernsteinValue(
                        ctrl_pts_y, time_seq[i],
                        primitive[tracker_idx][feasible_index[tracker_idx][sample_step * idx]].t0,
                        primitive[tracker_idx][sample_step * idx].tf, 3);
                temp_point.z = getBernsteinValue(
                        ctrl_pts_z, time_seq[i],
                        primitive[tracker_idx][feasible_index[tracker_idx][sample_step * idx]].t0,
                        primitive[tracker_idx][sample_step * idx].tf, 3);
                feasible_primitive_.points.push_back(temp_point);
            }
            visual_output.markers.push_back(feasible_primitive_);
        }
        { // ERASE
            const static int num_tracker = primitive.size();
            static vector<int> last_num_id(num_tracker);
            static bool once_flag = true;
            if (once_flag) {
                for (int ii = 0; ii < num_tracker; ii++)
                    last_num_id[ii] = 2000;
                once_flag = false;
            }
            visualization_msgs::Marker erase_marker;
            erase_marker.action = visualization_msgs::Marker::DELETE;
            erase_marker.ns = "TRACKER_FEASIBLE_" + std::to_string(tracker_idx);
            erase_marker.header.frame_id = param_.frame_id;
            for (int i = num_primitive_vis - 1; i <= last_num_id[tracker_idx]; i++) {
                erase_marker.id = i;
                visual_output.markers.push_back(erase_marker);
            }
            last_num_id[tracker_idx] = num_primitive_vis - 1;
        }
    }
    return visual_output;
}

visualization_msgs::MarkerArray
dmvc3d::PlanningVisualizer::VisualizeBestPrimitive(const vector<vector<dmvc3d::PrimitivePlanning>> &primitive,
                                                   const vector<uint> &best_index) {
    if (primitive.empty())
        return visualization_msgs::MarkerArray{};
    if (best_index.empty())
        return visualization_msgs::MarkerArray{};
    visualization_msgs::MarkerArray visual_output;
    for (int tracker_idx = 0; tracker_idx < primitive.size(); tracker_idx++) {
        std::vector<double> time_seq;
        double seg_t0 = primitive[tracker_idx][0].t0;
        double seg_tf = primitive[tracker_idx][0].tf;
        for (int i = 0; i < param_.best_primitive.num_time_sample; i++)
            time_seq.push_back(seg_t0 + (double) i * (seg_tf - seg_t0) /
                                        (double) (param_.best_primitive.num_time_sample - 1));
        geometry_msgs::Point temp_point;
        double ctrl_pts_x[4];
        double ctrl_pts_y[4];
        double ctrl_pts_z[4];
        best_primitive_.points.clear();
        best_primitive_.ns = "TRACKER_BEST_" + to_string(tracker_idx);
        best_primitive_.id = 0;
        for (int i = 0; i < 4; i++) {
            ctrl_pts_x[i] = primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[i];
            ctrl_pts_y[i] = primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[i];
            ctrl_pts_z[i] = primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[i];
        }
        for (int i = 0; i < time_seq.size(); i++) {
            temp_point.x = getBernsteinValue(
                    ctrl_pts_x, time_seq[i], primitive[tracker_idx][best_index[tracker_idx]].t0,
                    primitive[tracker_idx][best_index[tracker_idx]].tf, 3);
            temp_point.y = getBernsteinValue(
                    ctrl_pts_y, time_seq[i], primitive[tracker_idx][best_index[tracker_idx]].t0,
                    primitive[tracker_idx][best_index[tracker_idx]].tf, 3);
            temp_point.z = getBernsteinValue(
                    ctrl_pts_z, time_seq[i], primitive[tracker_idx][best_index[tracker_idx]].t0,
                    primitive[tracker_idx][best_index[tracker_idx]].tf, 3);
            best_primitive_.points.push_back(temp_point);
        }
        visual_output.markers.push_back(best_primitive_);
    }
    return visual_output;
}


void dmvc3d::PlanningVisualizer::UpdateParameter(const dmvc3d::VisualizationParam &param) {
    param_ = param;
    // RAW PRIMITIVE
    raw_primitive_.header.frame_id = param_.frame_id;
    raw_primitive_.type = visualization_msgs::Marker::LINE_STRIP;
    raw_primitive_.scale.x = param_.raw_primitives.line_scale;
    raw_primitive_.ns = "TRACKER_RAW";
    raw_primitive_.color.a = (float) param_.raw_primitives.color_a;
    raw_primitive_.color.r = (float) param_.raw_primitives.color_r;
    raw_primitive_.color.g = (float) param_.raw_primitives.color_g;
    raw_primitive_.color.b = (float) param_.raw_primitives.color_b;
    raw_primitive_.pose.orientation.w = 1.0;
    raw_primitive_.pose.orientation.x = 0.0;
    raw_primitive_.pose.orientation.y = 0.0;
    raw_primitive_.pose.orientation.z = 0.0;
    // FEASIBLE PRIMITIVE
    feasible_primitive_.header.frame_id = param_.frame_id;
    feasible_primitive_.type = visualization_msgs::Marker::LINE_STRIP;
    feasible_primitive_.scale.x = param_.feasible_primitives.line_scale;
    feasible_primitive_.ns = "TRACKER_FEASIBLE";
    feasible_primitive_.color.a = (float) param_.feasible_primitives.color_a;
    feasible_primitive_.color.r = (float) param_.feasible_primitives.color_r;
    feasible_primitive_.color.g = (float) param_.feasible_primitives.color_g;
    feasible_primitive_.color.b = (float) param_.feasible_primitives.color_b;
    feasible_primitive_.pose.orientation.w = 1.0;
    feasible_primitive_.pose.orientation.x = 0.0;
    feasible_primitive_.pose.orientation.y = 0.0;
    feasible_primitive_.pose.orientation.z = 0.0;
    // BEST PRIMITIVE
    best_primitive_.header.frame_id = param_.frame_id;
    best_primitive_.type = visualization_msgs::Marker::LINE_STRIP;
    best_primitive_.scale.x = param_.best_primitive.line_scale;
    best_primitive_.ns = "TRACKER_BEST";
    best_primitive_.color.a = (float) param_.best_primitive.color_a;
    best_primitive_.color.r = (float) param_.best_primitive.color_r;
    best_primitive_.color.g = (float) param_.best_primitive.color_g;
    best_primitive_.color.b = (float) param_.best_primitive.color_b;
    best_primitive_.pose.orientation.w = 1.0;
    best_primitive_.pose.orientation.x = 0.0;
    best_primitive_.pose.orientation.y = 0.0;
    best_primitive_.pose.orientation.z = 0.0;
}

dmvc3d::PlanningVisualizer::PlanningVisualizer(const dmvc3d::VisualizationParam &vis_param) {
    param_ = vis_param;
    // RAW PRIMITIVE
    raw_primitive_.header.frame_id = param_.frame_id;
    raw_primitive_.type = visualization_msgs::Marker::LINE_STRIP;
    raw_primitive_.scale.x = param_.raw_primitives.line_scale;
    raw_primitive_.ns = "TRACKER_RAW";
    raw_primitive_.color.a = (float) param_.raw_primitives.color_a;
    raw_primitive_.color.r = (float) param_.raw_primitives.color_r;
    raw_primitive_.color.g = (float) param_.raw_primitives.color_g;
    raw_primitive_.color.b = (float) param_.raw_primitives.color_b;
    raw_primitive_.pose.orientation.w = 1.0;
    raw_primitive_.pose.orientation.x = 0.0;
    raw_primitive_.pose.orientation.y = 0.0;
    raw_primitive_.pose.orientation.z = 0.0;
    // FEASIBLE PRIMITIVE
    feasible_primitive_.header.frame_id = param_.frame_id;
    feasible_primitive_.type = visualization_msgs::Marker::LINE_STRIP;
    feasible_primitive_.scale.x = param_.feasible_primitives.line_scale;
    feasible_primitive_.ns = "TRACKER_FEASIBLE";
    feasible_primitive_.color.a = (float) param_.feasible_primitives.color_a;
    feasible_primitive_.color.r = (float) param_.feasible_primitives.color_r;
    feasible_primitive_.color.g = (float) param_.feasible_primitives.color_g;
    feasible_primitive_.color.b = (float) param_.feasible_primitives.color_b;
    feasible_primitive_.pose.orientation.w = 1.0;
    feasible_primitive_.pose.orientation.x = 0.0;
    feasible_primitive_.pose.orientation.y = 0.0;
    feasible_primitive_.pose.orientation.z = 0.0;
    // BEST PRIMITIVE
    best_primitive_.header.frame_id = param_.frame_id;
    best_primitive_.type = visualization_msgs::Marker::LINE_STRIP;
    best_primitive_.scale.x = param_.best_primitive.line_scale;
    best_primitive_.ns = "TRACKER_BEST";
    best_primitive_.color.a = (float) param_.best_primitive.color_a;
    best_primitive_.color.r = (float) param_.best_primitive.color_r;
    best_primitive_.color.g = (float) param_.best_primitive.color_g;
    best_primitive_.color.b = (float) param_.best_primitive.color_b;
    best_primitive_.pose.orientation.w = 1.0;
    best_primitive_.pose.orientation.x = 0.0;
    best_primitive_.pose.orientation.y = 0.0;
    best_primitive_.pose.orientation.z = 0.0;

    single_voronoi_cell_.header.frame_id = param_.frame_id;
    single_voronoi_cell_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    single_voronoi_cell_.pose.position = GetDefaultPointMsg();
    single_voronoi_cell_.pose.orientation = GetDefaultQuaternionMsg();
    single_voronoi_cell_.scale = GetDefaultScaleMsg();
    single_voronoi_cell_.ns = "MOVING_VORONOI_CELL";
    single_voronoi_cell_.action = visualization_msgs::Marker::ADD;

    voronoi_cell_color_.a = (float) param_.cell.voronoi.color_a;
    voronoi_cell_color_.r = (float) param_.cell.voronoi.color_r;
    voronoi_cell_color_.g = (float) param_.cell.voronoi.color_g;
    voronoi_cell_color_.b = (float) param_.cell.voronoi.color_b;

    single_voronoi_cell_.color.a = voronoi_cell_color_.a;
    single_voronoi_cell_.color.r = voronoi_cell_color_.r;
    single_voronoi_cell_.color.g = voronoi_cell_color_.g;
    single_voronoi_cell_.color.b = voronoi_cell_color_.b;

    single_visibility_cell_.header.frame_id = param_.frame_id;
    single_visibility_cell_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    single_visibility_cell_.pose.position = GetDefaultPointMsg();
    single_visibility_cell_.pose.orientation = GetDefaultQuaternionMsg();
    single_visibility_cell_.scale = GetDefaultScaleMsg();
    single_visibility_cell_.ns = "MOVING_VISIBILITY_CELL";
    single_visibility_cell_.action = visualization_msgs::Marker::ADD;

    visibility_cell_color_.a = (float) param_.cell.visibility.color_a;
    visibility_cell_color_.r = (float) param_.cell.visibility.color_r;
    visibility_cell_color_.g = (float) param_.cell.visibility.color_g;
    visibility_cell_color_.b = (float) param_.cell.visibility.color_b;

    single_visibility_cell_.color.a = visibility_cell_color_.a;
    single_visibility_cell_.color.r = visibility_cell_color_.r;
    single_visibility_cell_.color.g = visibility_cell_color_.g;
    single_visibility_cell_.color.b = visibility_cell_color_.b;

}

visualization_msgs::Marker
dmvc3d::PlanningVisualizer::VisualizeConvexHull(const Vec3List &convex_hull, const int &tracker_idx,
                                                const uint &cell_type) {
    if (convex_hull.empty())
        return visualization_msgs::Marker{};
    if (cell_type == MOVING_BUFFERED_VORONOI_CELL) {
        single_voronoi_cell_.colors.clear();
        single_voronoi_cell_.points.clear();
        single_voronoi_cell_.id = tracker_idx;
    }
    if (cell_type == MOVING_VISIBILITY_CELL) {
        single_visibility_cell_.colors.clear();
        single_visibility_cell_.points.clear();
        single_visibility_cell_.id = tracker_idx;
    }

    size_t num_vertices = convex_hull.size();
    ch_vertex *vertices;
    vertices = (ch_vertex *) malloc(num_vertices * sizeof(ch_vertex));
    for (size_t i = 0; i < num_vertices; i++) {
        vertices[i].x = convex_hull[i].x();
        vertices[i].y = convex_hull[i].y();
        vertices[i].z = convex_hull[i].z();
    }
    int *face_indices = nullptr;
    int num_faces;
    convhull_3d_build(vertices, num_vertices, &face_indices, &num_faces);
    if (cell_type == MOVING_BUFFERED_VORONOI_CELL) {
        PointMsg vertex;
        for (int i = 0; i < num_faces; i++) {
            auto vertex1 = vertices[face_indices[i * 3]];
            auto vertex2 = vertices[face_indices[i * 3 + 1]];
            auto vertex3 = vertices[face_indices[i * 3 + 2]];
            vertex.x = vertex1.x, vertex.y = vertex1.y, vertex.z = vertex1.z;
            single_voronoi_cell_.points.emplace_back(vertex);
            vertex.x = vertex2.x, vertex.y = vertex2.y, vertex.z = vertex2.z;
            single_voronoi_cell_.points.emplace_back(vertex);
            vertex.x = vertex3.x, vertex.y = vertex3.y, vertex.z = vertex3.z;
            single_voronoi_cell_.points.emplace_back(vertex);
            single_voronoi_cell_.colors.push_back(voronoi_cell_color_);
        }
    }
    if (cell_type == MOVING_VISIBILITY_CELL) {
        PointMsg vertex;
        for (int i = 0; i < num_faces; i++) {
            auto vertex1 = vertices[face_indices[i * 3]];
            auto vertex2 = vertices[face_indices[i * 3 + 1]];
            auto vertex3 = vertices[face_indices[i * 3 + 2]];
            vertex.x = vertex1.x, vertex.y = vertex1.y, vertex.z = vertex1.z;
            single_visibility_cell_.points.emplace_back(vertex);
            vertex.x = vertex2.x, vertex.y = vertex2.y, vertex.z = vertex2.z;
            single_visibility_cell_.points.emplace_back(vertex);
            vertex.x = vertex3.x, vertex.y = vertex3.y, vertex.z = vertex3.z;
            single_visibility_cell_.points.emplace_back(vertex);
            single_visibility_cell_.colors.push_back(visibility_cell_color_);
        }
    }

    free(vertices);
    free(face_indices);
    if (cell_type == MOVING_BUFFERED_VORONOI_CELL)
        return single_voronoi_cell_;
    if (cell_type == MOVING_VISIBILITY_CELL)
        return single_visibility_cell_;
}

visualization_msgs::MarkerArray dmvc3d::PlanningVisualizer::VisualizeBufferedVoronoiCell(
        const std::vector<std::vector<dmvc3d::AffineCoeff2D>> &buffered_voronoi_cell) {
    visualization_msgs::MarkerArray visual_output;
    if (buffered_voronoi_cell.empty())
        return visual_output;
    int num_tracker = buffered_voronoi_cell.size();
    for (int tracker_idx = 0; tracker_idx < num_tracker; tracker_idx++) {
        Vec3List vertices = GetFeasibleVertices(buffered_voronoi_cell[tracker_idx]);
        visual_output.markers.push_back(VisualizeConvexHull(vertices, tracker_idx, MOVING_BUFFERED_VORONOI_CELL));
    }
    return visual_output;
}

std::vector<dmvc3d::AffineCoeff3D> dmvc3d::PlanningVisualizer::GetHalfSpaceFromBoundary() {
    vector<dmvc3d::AffineCoeff3D> boundary_halfspaces;
    dmvc3d::AffineCoeff3D half_space;
    {   // x-limit
        half_space[0] = -1.0;
        half_space[1] = 0.0;
        half_space[2] = 0.0;
        half_space[3] = param_.cell.axis_min.x;
        boundary_halfspaces.push_back(half_space);
        half_space[0] = 1.0;
        half_space[1] = 0.0;
        half_space[2] = 0.0;
        half_space[3] = -param_.cell.axis_max.x;
        boundary_halfspaces.push_back(half_space);
    }
    {   // y-limit
        half_space[0] = 0.0;
        half_space[1] = -1.0;
        half_space[2] = 0.0;
        half_space[3] = param_.cell.axis_min.y;
        boundary_halfspaces.push_back(half_space);
        half_space[0] = 0.0;
        half_space[1] = 1.0;
        half_space[2] = 0.0;
        half_space[3] = -param_.cell.axis_max.y;
        boundary_halfspaces.push_back(half_space);
    }
    {   // z-limit
        half_space[0] = 0.0;
        half_space[1] = 0.0;
        half_space[2] = -1.0;
        half_space[3] = param_.cell.axis_min.z;
        boundary_halfspaces.push_back(half_space);
        half_space[0] = 0.0;
        half_space[1] = 0.0;
        half_space[2] = 1.0;
        half_space[3] = -param_.cell.axis_max.z;
        boundary_halfspaces.push_back(half_space);
    }
    return boundary_halfspaces;
}

Vec3List dmvc3d::PlanningVisualizer::GetFeasibleVertices(const std::vector<dmvc3d::AffineCoeff2D> &constraint) {
    Vec3List vertices;
    if (constraint.empty())
        return vertices;
    dmvc3d::AffineCoeff3D single_constraint;
    vector<dmvc3d::AffineCoeff3D> constraint_list;

    for (int idx = 0; idx < constraint.size(); idx++) {
        single_constraint[0] = constraint[idx][0];
        single_constraint[1] = constraint[idx][1];
        single_constraint[2] = 0.0;
        single_constraint[3] = constraint[idx][2];
        constraint_list.push_back(single_constraint);
    }
    vector<dmvc3d::AffineCoeff3D> world_boundary_constraint = GetHalfSpaceFromBoundary();
    for (int i = 0; i < world_boundary_constraint.size(); i++)
        constraint_list.push_back(world_boundary_constraint[i]);

    Eigen::Matrix3Xd vertices_mtx;
    Eigen::MatrixX4d hs_mtx(constraint_list.size(), 4);
    int row = 0;
    for (const auto &h: constraint_list) {
        // h = {x | n \dot (x - p) - d >= 0}
        // h1*x + h2*y +h3*z+ h4 <=0
        hs_mtx(row, 0) = h[0];
        hs_mtx(row, 1) = h[1];
        hs_mtx(row, 2) = h[2];
        hs_mtx(row, 3) = h[3];
        row++;
    }
    if (not geo_utils::enumerateVs(hs_mtx, vertices_mtx)) {
        bool success = geo_utils::enumerateVs(hs_mtx, vertices_mtx);
        if (not success) {
            std::cout << "[Constraints]: Fail to find feasible vertices!" << std::endl;
            return vertices;
        }
    }
    for (int i = 0; i < vertices_mtx.cols(); i++)
        vertices.emplace_back(vertices_mtx.col(i)(0), vertices_mtx.col(i)(1), vertices_mtx.col(i)(2));

    return vertices;
}

visualization_msgs::MarkerArray dmvc3d::PlanningVisualizer::VisualizeVisibilityCell(
        const std::vector<std::vector<dmvc3d::AffineCoeff2D>> &visibility_cell) {
    visualization_msgs::MarkerArray visual_output;
    if (visibility_cell.empty())
        return visual_output;
    int num_tracker = visibility_cell.size();
//    cout<<"VISIBILITY CELL: "<<visibility_cell.size()<<endl;
    for (int tracker_idx = 0; tracker_idx < num_tracker; tracker_idx++) {
        Vec3List vertices = GetFeasibleVertices(visibility_cell[tracker_idx]);
//        cout<<tracker_idx<<"-th agent's cell Vertices Check"<<endl;
        visual_output.markers.push_back(VisualizeConvexHull(vertices, tracker_idx, MOVING_VISIBILITY_CELL));
//        cout<<tracker_idx<<"-th agent's cell ConvexHullMaking"<<endl;
    }
}