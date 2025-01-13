//
// Created by larr-planning on 25. 1. 13.
//
#include<dmvc_tracker3d/RosWrapper.h>

void dmvc3d::RosWrapper::RunRos() {
    ros::Rate loop_rate(this->GetControlFrequency());
    ros::AsyncSpinner spinner(4);
    spinner.start();
    while (ros::ok()) {
        PrepareRosMsgs();
        PublishRosMsgs();
        ros::spinOnce();
        loop_rate.sleep();
    }
}


void dmvc3d::RosWrapper::PrepareRosMsgs() {

}

void dmvc3d::RosWrapper::PublishRosMsgs() {
//    cout<<"1111111111"<<endl;
    { // RAW
        p_base_->mutex_set_[1].lock();
        tracker_raw_primitives_publisher_.publish(
                visualizer_->VisualizeRawPrimitives(p_base_->tracker_raw_primitives_));
        p_base_->mutex_set_[1].unlock();
    }
//    cout<<"22222222222"<<endl;
    {   // FEASIBLE
        p_base_->mutex_set_[1].lock();
        tracker_feasible_primitives_publisher_.publish(
                visualizer_->VisualizeFeasiblePrimitives(p_base_->tracker_raw_primitives_,
                                                         p_base_->tracker_feasible_index_));
        p_base_->mutex_set_[1].unlock();
    }
//    cout<<"3333333333333"<<endl;
    {   // BEST
        p_base_->mutex_set_[1].lock();
        tracker_best_trajectory_publisher_.publish(
                visualizer_->VisualizeBestPrimitive(p_base_->tracker_raw_primitives_, p_base_->tracker_best_index_));
        p_base_->mutex_set_[1].unlock();
    }
//    cout<<"4444444444444"<<endl;
    {   // CONTROL INPUT
        p_base_->mutex_set_[1].lock();
        if(p_base_->success_flag_)
            tracker_control_input_publisher_.publish(GenerateControlInput(p_base_->tracker_raw_primitives_,p_base_->tracker_best_index_,this->GetCurrentTime()));
        p_base_->mutex_set_[1].unlock();
    }
//    cout<<"5555555555555"<<endl;
    {   // VORONOI CELL
        p_base_->mutex_set_[1].lock();
        if(p_base_->success_flag_)
            buffered_voronoi_cell_publisher_.publish(visualizer_->VisualizeBufferedVoronoiCell(p_base_->buffered_voronoi_cell_));
        p_base_->mutex_set_[1].unlock();
    }
//    cout<<"6666666666666"<<endl;
//    {   // Visibility Cell
//        p_base_->mutex_set_[1].lock();
//        if(p_base_->success_flag_)
//            visibility_cell_publisher_.publish(visualizer_->VisualizeVisibilityCell(p_base_->visibility_cell_));
//        p_base_->mutex_set_[1].unlock();
//    }
//    cout<<"77777777777"<<endl;
//    {   // Corridor
//        p_base_->mutex_set_[1].lock();
//        if(not p_base_->polys_.empty()){
//            decomp_ros_msgs::PolyhedronArray polyhedron_msg = DecompROS::polyhedron_array_to_ros(p_base_->polys_);
//            polyhedron_msg.header.frame_id = vis_param_.frame_id;
//            corridor_publisher_.publish(polyhedron_msg);
//        }
//        p_base_->mutex_set_[1].unlock();
//    }
}

dmvc3d::TrackingParam dmvc3d::RosWrapper::GetTrackingParam() {
    return planning_param_;
}

dmvc3d::VisualizationParam dmvc3d::RosWrapper::GetVisualizationParam() {
    return vis_param_;
}

void dmvc3d::RosWrapper::InitSubscriberAndPublisher() {
    // Subscribe Problem Materials
    target_prediction_subscriber_ = nh_.subscribe("/dmvc_predictor3d/target_prediction", 1,
                                                  &RosWrapper::TargetPredictionCallback, this);
    obstacle_state_list_subscriber_ = nh_.subscribe("/dmvc_simulator3d/obstacle_state_list", 1,
                                                    &RosWrapper::ObstacleStateListCallback, this);
    tracker_list_state_subscriber_ = nh_.subscribe("/dmvc_simulator3d/tracker_state_list", 1,
                                                   &RosWrapper::TrackerStateListCallback, this);
    pcl_subscriber_ = nh_.subscribe("/dmvc_simulator3d/point_cloud_obstacle", 1, &RosWrapper::PclCallback, this);
    corridor_publisher_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("corridor", 1);

    // Publish Primitives
    tracker_raw_primitives_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("raw_primitive", 1);
    tracker_feasible_primitives_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("feasible_primitives", 1);
    tracker_best_trajectory_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("best_primitives", 1);
    tracker_control_input_publisher_ = nh_.advertise<dmvc_tracker3d::ControlInputList>("tracker_control_input", 1);
    buffered_voronoi_cell_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("buffered_voronoi_cell",1);
    visibility_cell_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visibility_cell",1);
//    tracker_raw_primitives_publisher_ = nh_.advertise<
}

void dmvc3d::RosWrapper::ObstacleStateListCallback(const dmvc_tracker3d::ObjectStateList &msg) {
    dmvc3d::State obstacle_state;
    p_base_->mutex_set_[0].lock();
    p_base_->current_obstacle_list_read_.clear();
    for (int i = 0; i < msg.object_state_list.size(); i++) {
        obstacle_state.px = msg.object_state_list[i].px;
        obstacle_state.py = msg.object_state_list[i].py;
        obstacle_state.pz = msg.object_state_list[i].pz;
        obstacle_state.vx = msg.object_state_list[i].vx;
        obstacle_state.vy = msg.object_state_list[i].vy;
        obstacle_state.vz = msg.object_state_list[i].vz;
        p_base_->current_obstacle_list_read_.push_back(obstacle_state);
    }
    p_base_->is_obstacle_info_ = true;
    p_base_->mutex_set_[0].unlock();
}

void dmvc3d::RosWrapper::TrackerStateListCallback(const dmvc_tracker3d::ObjectStateList &msg) {
    dmvc3d::State tracker_state;
    p_base_->mutex_set_[0].lock();
    p_base_->current_tracker_list_read_.clear();
    for (int i = 0; i < msg.object_state_list.size(); i++) {
        tracker_state.px = msg.object_state_list[i].px;
        tracker_state.py = msg.object_state_list[i].py;
        tracker_state.pz = msg.object_state_list[i].pz;
        tracker_state.vx = msg.object_state_list[i].vx;
        tracker_state.vy = msg.object_state_list[i].vy;
        tracker_state.vz = msg.object_state_list[i].vz;
        p_base_->current_tracker_list_read_.push_back(tracker_state);
    }
    p_base_->is_tracker_info_ = true;
    p_base_->mutex_set_[0].unlock();
//    printf("TRACKER LIST READ CALLED\n");
}

void dmvc3d::RosWrapper::TargetPredictionCallback(const dmvc_tracker3d::PolyState &msg) {
    dmvc3d::PrimitivePlanning target_trajectory;
    p_base_->mutex_set_[0].lock();
    p_base_->target_prediction_read_.t0 = msg.t0;
    p_base_->target_prediction_read_.tf = msg.tf;
    for (int i = 0; i < 4; i++) {
        p_base_->target_prediction_read_.ctrl_x[i] = msg.x_coeff[i];
        p_base_->target_prediction_read_.ctrl_y[i] = msg.y_coeff[i];
        p_base_->target_prediction_read_.ctrl_z[i] = msg.z_coeff[i];
    }
    p_base_->is_target_info_ = true;
    p_base_->mutex_set_[0].unlock();
//    printf("TARGET PREDICTION READ CALLED\n");
}

dmvc3d::RosWrapper::RosWrapper(std::shared_ptr<dmvc3d::PlannerBase> p_base) : p_base_(p_base), nh_("~") {
    t0_ = ros::Time::now().toSec();
    // ROS PARAM
    nh_.param<double>("control_frequency", ros_param_.control_frequency, 100.0);
    nh_.param<double>("planning_frequency", ros_param_.planning_frequency, 30.0);

    // VISUALIZATION PARAM
    nh_.param<std::string>("map_frame_id", vis_param_.frame_id, "map");
    nh_.param<bool>("raw_primitives/publish", vis_param_.raw_primitives.publish, "false");
    nh_.param<int>("raw_primitives/num_time_sample", vis_param_.raw_primitives.num_time_sample, 10);
    nh_.param<double>("raw_primitives/proportion", vis_param_.raw_primitives.proportion, 0.0);
    nh_.param<double>("raw_primitives/line_scale", vis_param_.raw_primitives.line_scale, 0.01);
    nh_.param<double>("raw_primitives/color_a", vis_param_.raw_primitives.color_a, 0.);
    nh_.param<double>("raw_primitives/color_r", vis_param_.raw_primitives.color_r, 0.);
    nh_.param<double>("raw_primitives/color_g", vis_param_.raw_primitives.color_g, 0.);
    nh_.param<double>("raw_primitives/color_b", vis_param_.raw_primitives.color_b, 0.);
    // Feasible primitives
    nh_.param<bool>("feasible_primitives/publish", vis_param_.feasible_primitives.publish, "false");
    nh_.param<int>("feasible_primitives/num_time_sample", vis_param_.feasible_primitives.num_time_sample, 10);
    nh_.param<double>("feasible_primitives/proportion", vis_param_.feasible_primitives.proportion, 0.0);
    nh_.param<double>("feasible_primitives/line_scale", vis_param_.feasible_primitives.line_scale, 0.01);
    nh_.param<double>("feasible_primitives/color_a", vis_param_.feasible_primitives.color_a, 0.);
    nh_.param<double>("feasible_primitives/color_r", vis_param_.feasible_primitives.color_r, 0.);
    nh_.param<double>("feasible_primitives/color_g", vis_param_.feasible_primitives.color_g, 0.);
    nh_.param<double>("feasible_primitives/color_b", vis_param_.feasible_primitives.color_b, 0.);
    // Best primitive
    nh_.param<int>("best_primitive/num_time_sample", vis_param_.best_primitive.num_time_sample, 10);
    nh_.param<double>("best_primitive/line_scale", vis_param_.best_primitive.line_scale, 0.01);
    nh_.param<double>("best_primitive/color_a", vis_param_.best_primitive.color_a, 0.);
    nh_.param<double>("best_primitive/color_r", vis_param_.best_primitive.color_r, 0.);
    nh_.param<double>("best_primitive/color_g", vis_param_.best_primitive.color_g, 0.);
    nh_.param<double>("best_primitive/color_b", vis_param_.best_primitive.color_b, 0.);
    // Cell
    nh_.param<double>("cell/min_x",vis_param_.cell.axis_min.x,0.0);
    nh_.param<double>("cell/min_y",vis_param_.cell.axis_min.y,0.0);
    nh_.param<double>("cell/min_z",vis_param_.cell.axis_min.z,0.0);
    nh_.param<double>("cell/max_x",vis_param_.cell.axis_max.x,0.0);
    nh_.param<double>("cell/max_y",vis_param_.cell.axis_max.y,0.0);
    nh_.param<double>("cell/max_z",vis_param_.cell.axis_max.z,0.0);
    // Voronoi
    nh_.param<double>("cell/voronoi/color_a",vis_param_.cell.voronoi.color_a,0.0);
    nh_.param<double>("cell/voronoi/color_r",vis_param_.cell.voronoi.color_r,0.0);
    nh_.param<double>("cell/voronoi/color_g",vis_param_.cell.voronoi.color_g,0.0);
    nh_.param<double>("cell/voronoi/color_b",vis_param_.cell.voronoi.color_b,0.0);
    // Visibility
    nh_.param<double>("cell/visibility/color_a",vis_param_.cell.visibility.color_a,0.0);
    nh_.param<double>("cell/visibility/color_r",vis_param_.cell.visibility.color_r,0.0);
    nh_.param<double>("cell/visibility/color_g",vis_param_.cell.visibility.color_g,0.0);
    nh_.param<double>("cell/visibility/color_b",vis_param_.cell.visibility.color_b,0.0);

    visualizer_ = new PlanningVisualizer(vis_param_);
    // Planning
    nh_.param<double>("horizon", planning_param_.horizon, 1.0);
    nh_.param<int>("num_thread", planning_param_.num_thread, 1);
    nh_.param<double>("r_min", planning_param_.r_min, 1.0);
    nh_.param<double>("r_max", planning_param_.r_max, 2.0);
    nh_.param<double>("elevation_range",planning_param_.elevation_range,0.05);
    nh_.param<double>("safe_distance", planning_param_.safe_distance, 0.0);

    nh_.param<int>("num_sample_planning", planning_param_.num_sample_planning, 10);
    nh_.param<double>("vel_max", planning_param_.vel_max, 0.5);
    nh_.param<double>("acc_max", planning_param_.acc_max, 10.0);
    nh_.param<double>("object_radius", planning_param_.object_radius, 0.15);
    nh_.param<double>("terminal_weight", planning_param_.terminal_weight, 0.0);
    nh_.param<double>("distance_max", planning_param_.distance_max, 1.0);
    nh_.param<bool>("is_experiment", planning_param_.is_experiment, false);
    nh_.param<bool>("is_unstructured",planning_param_.is_unstructured, false);

    nh_.param<double>("axis_limit/min_x", planning_param_.axis_limit.min_x, 0.0);
    nh_.param<double>("axis_limit/min_y", planning_param_.axis_limit.min_y, 0.0);
    nh_.param<double>("axis_limit/max_x", planning_param_.axis_limit.max_x, 0.0);
    nh_.param<double>("axis_limit/max_y", planning_param_.axis_limit.max_y, 0.0);
}

dmvc_tracker3d::ControlInputList
dmvc3d::RosWrapper::GenerateControlInput(const std::vector<std::vector<dmvc3d::PrimitivePlanning>> &primitive,
                                       const std::vector<uint> &best_index, const double &time_t) {
    dmvc_tracker3d::ControlInput tracker_input;
    dmvc_tracker3d::ControlInputList tracker_input_list;
    int num_tracker = best_index.size();
    for (int tracker_idx = 0; tracker_idx < num_tracker; tracker_idx++) {
        double T =
                primitive[tracker_idx][best_index[tracker_idx]].tf - primitive[tracker_idx][best_index[tracker_idx]].t0;
        double T2 = T * T;
        double T_inv = 1.0 / T;
        double T2_inv = 1.0 / T2;
        double pos_ctrl_x[4]{primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[0],
                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[1],
                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[2],
                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[3]};
        double pos_ctrl_y[4]{primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[0],
                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[1],
                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[2],
                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[3]};
        double pos_ctrl_z[4]{primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[0],
                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[1],
                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[2],
                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[3]};
        double vel_ctrl_x[3]{3.0 * T_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[1] -
                                            primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[0]),
                             3.0 * T_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[2] -
                                            primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[1]),
                             3.0 * T_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[3] -
                                            primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[2])};
        double vel_ctrl_y[3]{3.0 * T_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[1] -
                                            primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[0]),
                             3.0 * T_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[2] -
                                            primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[1]),
                             3.0 * T_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[3] -
                                            primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[2])};
        double vel_ctrl_z[3]{3.0 * T_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[1] -
                                            primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[0]),
                             3.0 * T_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[2] -
                                            primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[1]),
                             3.0 * T_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[3] -
                                            primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[2])};
        double acc_ctrl_x[2]{6.0 * T2_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[2] -
                                             2.0 * primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[1] +
                                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[0]),
                             6.0 * T2_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[3] -
                                             2.0 * primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[2] +
                                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_x[1])};
        double acc_ctrl_y[2]{6.0 * T2_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[2] -
                                             2.0 * primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[1] +
                                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[0]),
                             6.0 * T2_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[3] -
                                             2.0 * primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[2] +
                                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_y[1])};
        double acc_ctrl_z[2]{6.0 * T2_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[2] -
                                             2.0 * primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[1] +
                                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[0]),
                             6.0 * T2_inv * (primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[3] -
                                             2.0 * primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[2] +
                                             primitive[tracker_idx][best_index[tracker_idx]].ctrl_z[1])};
        tracker_input.px = dmvc3d::getBernsteinValue(pos_ctrl_x, time_t,
                                                   primitive[tracker_idx][best_index[tracker_idx]].t0,
                                                   primitive[tracker_idx][best_index[tracker_idx]].tf, 3);
        tracker_input.py = dmvc3d::getBernsteinValue(pos_ctrl_y, time_t,
                                                   primitive[tracker_idx][best_index[tracker_idx]].t0,
                                                   primitive[tracker_idx][best_index[tracker_idx]].tf, 3);
        tracker_input.pz = dmvc3d::getBernsteinValue(pos_ctrl_z, time_t,
                                                   primitive[tracker_idx][best_index[tracker_idx]].t0,
                                                   primitive[tracker_idx][best_index[tracker_idx]].tf, 3);
        tracker_input.vx = dmvc3d::getBernsteinValue(vel_ctrl_x, time_t,
                                                   primitive[tracker_idx][best_index[tracker_idx]].t0,
                                                   primitive[tracker_idx][best_index[tracker_idx]].tf, 2);
        tracker_input.vy = dmvc3d::getBernsteinValue(vel_ctrl_y, time_t,
                                                   primitive[tracker_idx][best_index[tracker_idx]].t0,
                                                   primitive[tracker_idx][best_index[tracker_idx]].tf, 2);
        tracker_input.vz = dmvc3d::getBernsteinValue(vel_ctrl_z, time_t,
                                                   primitive[tracker_idx][best_index[tracker_idx]].t0,
                                                   primitive[tracker_idx][best_index[tracker_idx]].tf, 2);
        tracker_input.ax = dmvc3d::getBernsteinValue(acc_ctrl_x, time_t,
                                                   primitive[tracker_idx][best_index[tracker_idx]].t0,
                                                   primitive[tracker_idx][best_index[tracker_idx]].tf, 1);
        tracker_input.ay = dmvc3d::getBernsteinValue(acc_ctrl_y, time_t,
                                                   primitive[tracker_idx][best_index[tracker_idx]].t0,
                                                   primitive[tracker_idx][best_index[tracker_idx]].tf, 1);
        tracker_input.az = dmvc3d::getBernsteinValue(acc_ctrl_z, time_t,
                                                   primitive[tracker_idx][best_index[tracker_idx]].t0,
                                                   primitive[tracker_idx][best_index[tracker_idx]].tf, 1);
        tracker_input_list.control_input_list.push_back(tracker_input);

    }
    return tracker_input_list;
}

void dmvc3d::RosWrapper::PclCallback(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr &pcl_msgs) {
    p_base_->mutex_set_[0].lock();
    p_base_->point_cloud_3d_.clear();
    sensor_msgs::PointCloud pcl;
    if (not pcl_msgs->fields.empty()) {
        sensor_msgs::convertPointCloud2ToPointCloud(*pcl_msgs, pcl);
        pcl.header.frame_id = pcl_msgs->header.frame_id;
        pcl.header.stamp = pcl_msgs->header.stamp;
    }
    vec_Vec3f obs = DecompROS::cloud_to_vec(pcl);
    for (const auto &it: obs)
        p_base_->point_cloud_3d_.push_back(it.topRows<3>());
    p_base_->is_obstacle_info_ = true;
    p_base_->mutex_set_[0].unlock();
//    cout<<"PCLPCLPCLPCLPCL"<<endl;
}
