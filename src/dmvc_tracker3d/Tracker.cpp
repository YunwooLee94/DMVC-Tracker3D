//
// Created by larr-planning on 25. 1. 13.
//
#include <dmvc_tracker3d/Tracker.h>

bool dmvc3d::Tracker::Plan(const double &t_trigger) {
    if (CheckInfoAvailable())
        UpdateValue(t_trigger);
    else
        return false;
//    cout<<"START SAMPLE"<<endl;
    SampleEndPoint();
//    cout<<"GENERATE PRIMITIVES"<<endl;
    GeneratePrimitive(t_trigger);
//    cout<<"CHECK FEASIBILITY"<<endl;
    GetSafeIndex(); // Target Distance + Collision and Occlusion Avoidance against Obstacles
    bool pass_test1 = true;
    for (int idx = 0; idx < num_tracker_; idx++) {
        if (safe_index_[idx].empty()) {
            pass_test1 = false;
            printf("AT TIME %f [s], %d-th TRACKER FAIL DUE TO OBSTACLE\n", t_trigger, idx);
        }
    }
    if (pass_test1) {
//        cout<<"INTER SAFETY"<<endl;
        GetInterSafetyIndex();
        CalculateMovingBufferedVoronoiCell();
    } else
        return false;
    bool pass_test2 = true;
    for (int idx = 0; idx < num_tracker_; idx++) {
        if (inter_safe_index_[idx].empty()) {
            pass_test2 = false;
            printf("AT TIME %f [s], %d-th TRACKER FAIL DUE TO INTER-COLLISION\n", t_trigger, idx);
        }
    }
//    if (pass_test2) {
////        cout<<"INTER VISIBILITY"<<endl;
//        GetInterVisibleIndex();
//    } else
//        return false;
//    bool pass_test3 = true;
//    for (int idx = 0; idx < num_tracker_; idx++) {
//        if (inter_visible_index_[idx].empty()) {
//            pass_test3 = false;
//            printf("AT TIME %f [s], %d-th TRACKER FAIL DUE TO INTER-OCCLUSION\n", t_trigger, idx);
//        }
//    }
    if (pass_test2){
        GetDynamicallyFeasibleIndex();
    }
    else
        return false;
    bool pass_test4 = true;
    for (int idx = 0; idx < num_tracker_; idx++) {
        if (dynamically_feasible_index_[idx].empty()) {
            pass_test4 = false;
            printf("AT TIME %f [s], %d-th TRACKER FAIL DUE TO DYNAMIC LIMITS\n", t_trigger, idx);
        }
    }
    if (pass_test4){
//        cout<<"BEST INDEX"<<endl;
        GetBestIndex();
    }

    // Dynamic Constraints
    return pass_test4;
}

dmvc3d::Tracker::Tracker(const dmvc3d::TrackingParam &param, std::shared_ptr<dmvc3d::PlannerBase> p_base) : param_(param),
                                                                                                            p_base_(p_base) {
}

void dmvc3d::Tracker::UpdateValue(const double &t) {
    {   // Tracker State
        current_tracker_state_.clear();
        dmvc3d::State temp_state;
        p_base_->mutex_set_[0].lock();
        num_tracker_ = (int) p_base_->current_tracker_list_read_.size();
        for (int i = 0; i < num_tracker_; i++) {
            temp_state = p_base_->current_tracker_list_read_[i];
            current_tracker_state_.push_back(temp_state);
        }
        p_base_->mutex_set_[0].unlock();
    }
    {   // Obstacle State
        obstacle_primitive_list_.clear();
        dmvc3d::PrimitivePlanning temp_primitive;
        temp_primitive.t0 = t;
        temp_primitive.tf = t + param_.horizon;
        p_base_->mutex_set_[0].lock();
        num_obstacle_ = (int) p_base_->current_obstacle_list_read_.size();
        for (int i = 0; i < num_obstacle_; i++) {
            temp_primitive.ctrl_x[0] = p_base_->current_obstacle_list_read_[i].px;
            temp_primitive.ctrl_x[1] = p_base_->current_obstacle_list_read_[i].px +
                                       0.33333333 * p_base_->current_obstacle_list_read_[i].vx * param_.horizon;
            temp_primitive.ctrl_x[2] = p_base_->current_obstacle_list_read_[i].px +
                                       0.66666667 * p_base_->current_obstacle_list_read_[i].vx * param_.horizon;
            temp_primitive.ctrl_x[3] = p_base_->current_obstacle_list_read_[i].px +
                                       p_base_->current_obstacle_list_read_[i].vx * param_.horizon;
            temp_primitive.ctrl_y[0] = p_base_->current_obstacle_list_read_[i].py;
            temp_primitive.ctrl_y[1] = p_base_->current_obstacle_list_read_[i].py +
                                       0.33333333 * p_base_->current_obstacle_list_read_[i].vy * param_.horizon;
            temp_primitive.ctrl_y[2] = p_base_->current_obstacle_list_read_[i].py +
                                       0.66666667 * p_base_->current_obstacle_list_read_[i].vy * param_.horizon;
            temp_primitive.ctrl_y[3] = p_base_->current_obstacle_list_read_[i].py +
                                       p_base_->current_obstacle_list_read_[i].vy * param_.horizon;
            temp_primitive.ctrl_z[0] = p_base_->current_obstacle_list_read_[i].pz;
            temp_primitive.ctrl_z[1] = p_base_->current_obstacle_list_read_[i].pz +
                                       0.33333333 * p_base_->current_obstacle_list_read_[i].vz * param_.horizon;
            temp_primitive.ctrl_z[2] = p_base_->current_obstacle_list_read_[i].pz +
                                       0.66666667 * p_base_->current_obstacle_list_read_[i].vz * param_.horizon;
            temp_primitive.ctrl_z[3] = p_base_->current_obstacle_list_read_[i].pz +
                                       p_base_->current_obstacle_list_read_[i].vz * param_.horizon;
            obstacle_primitive_list_.push_back(temp_primitive);
        }
        p_base_->mutex_set_[0].unlock();
    }

    {   // Target State
        p_base_->mutex_set_[0].lock();
        target_trajectory_.t0 = t;
        target_trajectory_.tf = t + param_.horizon;
        for (int i = 0; i < 4; i++) {
            target_trajectory_.ctrl_x[i] = p_base_->target_prediction_read_.ctrl_x[i];
            target_trajectory_.ctrl_y[i] = p_base_->target_prediction_read_.ctrl_y[i];
            target_trajectory_.ctrl_z[i] = p_base_->target_prediction_read_.ctrl_z[i];
        }
        p_base_->mutex_set_[0].unlock();
    }
    {
        p_base_->mutex_set_[0].lock();
        point_cloud_3d_ = p_base_->point_cloud_3d_;
        p_base_->mutex_set_[0].unlock();
    }
}

bool dmvc3d::Tracker::CheckInfoAvailable() {
    p_base_->mutex_set_[0].lock();
    bool do_plan = p_base_->is_tracker_info_ and p_base_->is_target_info_ and p_base_->is_obstacle_info_;
    p_base_->mutex_set_[0].unlock();
    return do_plan;
}

void dmvc3d::Tracker::SampleEndPoint() {
    end_points_.clear();
    int num_chunk = param_.num_sample_planning / param_.num_thread;
    for (int i = 0; i < num_tracker_; i++) {
        vector<thread> worker_thread;
        vector<vector<dmvc3d::Point>> end_point_temp(param_.num_thread);
        vector<dmvc3d::Point> single_tracker_end_point;
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread.emplace_back(
                    thread(&Tracker::SampleEndPointThread, this, i, num_chunk * j, num_chunk * (j + 1),
                           std::ref(end_point_temp[j])));
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread[j].join();
        for (int j = 0; j < param_.num_thread; j++) {
            for (int k = 0; k < end_point_temp[j].size(); k++)
                single_tracker_end_point.push_back(end_point_temp[j][k]);
        }
        end_points_.push_back(single_tracker_end_point);
    }
}

void dmvc3d::Tracker::SampleEndPointThread(const int &tracker_idx, const int &start_idx, const int &end_idx,
                                           std::vector<dmvc3d::Point> &endpoint_list_sub) {
    dmvc3d::Point end_point_center{target_trajectory_.ctrl_x[3], target_trajectory_.ctrl_y[3],
                                   target_trajectory_.ctrl_z[3]};
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> r_dis(param_.r_min, param_.r_max);
    std::uniform_real_distribution<> elev_dis(-param_.elevation_range, param_.elevation_range);
    double center_angle = std::atan2(current_tracker_state_[tracker_idx].py - target_trajectory_.ctrl_y[3],
                                     current_tracker_state_[tracker_idx].px - target_trajectory_.ctrl_x[3]);
    double half_range = 1.0 * M_PI;
    std::uniform_real_distribution<> theta_dis(center_angle - half_range, center_angle + half_range);

    double r, theta, elevation;
    Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
    for (int i = start_idx; i < end_idx; i++) {
        r = r_dis(gen);
        theta = theta_dis(gen);
        elevation = elev_dis(gen);
        tempPoint.x = float(end_point_center.x + r * cos(theta));
        tempPoint.y = float(end_point_center.y + r * sin(theta));
        tempPoint.z = float(end_point_center.z + elevation);
        endpoint_list_sub.push_back(tempPoint);
    }
}

void dmvc3d::Tracker::GeneratePrimitive(const double &t) {
    primitive_.clear();
    int num_chunk = param_.num_sample_planning / param_.num_thread;
    for (int i = 0; i < num_tracker_; i++) {
        vector<thread> worker_thread;
        vector<vector<dmvc3d::PrimitivePlanning>> primitive_temp(param_.num_thread);
        vector<dmvc3d::PrimitivePlanning> single_tracker_primitives;
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread.emplace_back(
                    thread(&Tracker::GeneratePrimitiveThread, this, i, t, num_chunk * j, num_chunk * (j + 1),
                           std::ref(primitive_temp[j])));
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread[j].join();
        for (int j = 0; j < param_.num_thread; j++) {
            for (int k = 0; k < primitive_temp[j].size(); k++)
                single_tracker_primitives.push_back(primitive_temp[j][k]);
        }
        primitive_.push_back(single_tracker_primitives);
    }
}

void dmvc3d::Tracker::GetInterSafetyIndex() {
    inter_safe_index_.clear();
    for (int tracker_idx = 0; tracker_idx < num_tracker_; tracker_idx++) {
        int num_chunk = safe_index_[tracker_idx].size() / param_.num_thread;
//        printf("%d-th Tracker CHUNK SIZE: %d\n",tracker_idx,num_chunk);
        vector<thread> worker_thread;
        vector<vector<dmvc3d::uint>> inter_safe_index_temp(param_.num_thread);
        vector<dmvc3d::uint> single_inter_safe_index;
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread.emplace_back(
                    thread(&Tracker::GetInterSafetyIndexThread, this, tracker_idx, num_chunk * j, num_chunk * (j + 1),
                           std::ref(inter_safe_index_temp[j])));
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread[j].join();
        for (int j = 0; j < param_.num_thread; j++) {
            for (int k = 0; k < inter_safe_index_temp[j].size(); k++)
                single_inter_safe_index.push_back(inter_safe_index_temp[j][k]);
        }
        inter_safe_index_.push_back(single_inter_safe_index);
    }
}

void dmvc3d::Tracker::GetSafeIndex() {
    safe_index_.clear();
    if (param_.is_unstructured)
        GenerateCorridor();
    for (int idx = 0; idx < num_tracker_; idx++) {
        int num_chunk = primitive_[idx].size() / param_.num_thread;
        vector<thread> worker_thread;
        vector<vector<dmvc3d::uint>> safe_index_temp(param_.num_thread);
        vector<dmvc3d::uint> single_safe_index;
        if (param_.is_unstructured) {
            for (int j = 0; j < param_.num_thread; j++)
                worker_thread.emplace_back(
                        thread(&Tracker::GetSafeIndexUnstructuredThread, this, idx, num_chunk * j, num_chunk * (j + 1),
                               std::ref(safe_index_temp[j])));
        } else {
            for (int j = 0; j < param_.num_thread; j++)
                worker_thread.emplace_back(
                        thread(&Tracker::GetSafeIndexThread, this, idx, num_chunk * j, num_chunk * (j + 1),
                               std::ref(safe_index_temp[j])));
        }

        for (int j = 0; j < param_.num_thread; j++)
            worker_thread[j].join();
        for (int j = 0; j < param_.num_thread; j++) {
            for (int k = 0; k < safe_index_temp[j].size(); k++)
                single_safe_index.push_back(safe_index_temp[j][k]);
        }
        safe_index_.push_back(single_safe_index);
    }
}

void dmvc3d::Tracker::GetDynamicallyFeasibleIndex() {
    dynamically_feasible_index_.clear();
    inter_visible_index_ = inter_safe_index_;
    for (int idx = 0; idx < num_tracker_; idx++) {
        int num_chunk = inter_visible_index_[idx].size() / param_.num_thread;
        vector<thread> worker_thread;
        vector<vector<dmvc3d::uint>> dynamically_feasible_index_temp(param_.num_thread);
        vector<dmvc3d::uint> single_dynamically_feasible_index;
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread.emplace_back(
                    thread(&Tracker::GetDynamicallyFeasibleIndexThread, this, idx, num_chunk * j, num_chunk * (j + 1),
                           std::ref(dynamically_feasible_index_temp[j])));
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread[j].join();
        for (int j = 0; j < param_.num_thread; j++) {
            for (int k = 0; k < dynamically_feasible_index_temp[j].size(); k++)
                single_dynamically_feasible_index.push_back(dynamically_feasible_index_temp[j][k]);
        }
        dynamically_feasible_index_.push_back(single_dynamically_feasible_index);
    }
}

void dmvc3d::Tracker::GeneratePrimitiveThread(const int &tracker_idx, const double &t, const int &start_idx,
                                              const int &end_idx,
                                              std::vector<dmvc3d::PrimitivePlanning> &primitive_list_sub) {
    dmvc3d::PrimitivePlanning temp_primitive;
    temp_primitive.t0 = t;
    temp_primitive.tf = t + param_.horizon;
    double init_pos[3]{current_tracker_state_[tracker_idx].px, current_tracker_state_[tracker_idx].py,
                       current_tracker_state_[tracker_idx].pz};
    double init_vel[3]{current_tracker_state_[tracker_idx].vx, current_tracker_state_[tracker_idx].vy,
                       current_tracker_state_[tracker_idx].vz};
    double T = temp_primitive.tf - temp_primitive.t0;
    double T_3333 = 0.33333333 * T;
    for (int i = start_idx; i < end_idx; i++) {
        // x-coeff
        temp_primitive.ctrl_x[0] = init_pos[0];
        temp_primitive.ctrl_x[1] = init_pos[0] + T_3333 * init_vel[0];
        temp_primitive.ctrl_x[2] = 0.5 * init_pos[0] + 0.5 * end_points_[tracker_idx][i].x +
                                   0.16666667 * current_tracker_state_[tracker_idx].vx * T;
        temp_primitive.ctrl_x[3] = end_points_[tracker_idx][i].x;
        // y-coeff
        temp_primitive.ctrl_y[0] = init_pos[1];
        temp_primitive.ctrl_y[1] = init_pos[1] + T_3333 * init_vel[1];
        temp_primitive.ctrl_y[2] = 0.5 * init_pos[1] + 0.5 * end_points_[tracker_idx][i].y +
                                   0.16666667 * current_tracker_state_[tracker_idx].vy * T;
        temp_primitive.ctrl_y[3] = end_points_[tracker_idx][i].y;
        // z-coeff
        temp_primitive.ctrl_z[0] = init_pos[2];
        temp_primitive.ctrl_z[1] = init_pos[2] + T_3333 * init_vel[2];
        temp_primitive.ctrl_z[2] = 0.5 * init_pos[2] + 0.5 * end_points_[tracker_idx][i].z +
                                   0.16666667 * current_tracker_state_[tracker_idx].vz * T;
        temp_primitive.ctrl_z[3] = end_points_[tracker_idx][i].z;
        primitive_list_sub.push_back(temp_primitive);
    }
}

void dmvc3d::Tracker::GetDynamicallyFeasibleIndexThread(const int &tracker_idx, const int &start_idx, const int &end_idx,
                                                        std::vector<uint> &dyn_feas_idx_sub) {
    bool flag_store_velocity = true; // velocity
    bool flag_store_acceleration = true; // acceleration
    double value;
    double vel_x[3], vel_y[3], vel_z[3]; // velocity
    double acc_x[2], acc_y[2], acc_z[2]; // acceleration
    double T_inv = 1.0 / param_.horizon;
    double T2_inv = T_inv * T_inv;
    double vel_max_squared = param_.vel_max * param_.vel_max;
    double acc_max_squared = param_.acc_max * param_.acc_max;

    for (int idx = start_idx; idx < end_idx; idx++) {
        flag_store_velocity = true;
        for (int i = 0; i < 3; i++) {
            vel_x[i] = 3.0 * T_inv *
                       (primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_x[i + 1] -
                        primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_x[i]);
            vel_y[i] = 3.0 * T_inv *
                       (primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_y[i + 1] -
                        primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_y[i]);
            vel_z[i] = 3.0 * T_inv *
                       (primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_z[i + 1] -
                        primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_z[i]);
        }
        for (int j = 0; j <= 4; j++) {
            value = 0.0;
            for (int k = std::max(0, j - 2); k <= std::min(2, j); k++) {
                value +=
                        double(dmvc3d::nchooser(2, k)) * double(dmvc3d::nchooser(2, j - k)) / double(dmvc3d::nchooser(4, j)) *
                        (vel_x[k] * vel_x[j - k] + vel_y[k] * vel_y[j - k] + vel_z[k] * vel_z[j - k]);
            }
            if (value > vel_max_squared) {
                flag_store_velocity = false;
                break;
            }
        }
        if (not flag_store_velocity)
            continue;
        flag_store_acceleration = true;
        for (int i = 0; i < 2; i++) {
            acc_x[i] = 6.0 * T2_inv *
                       (primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_x[i + 2] -
                        2 * primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_x[i + 1] +
                        primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_x[i]);
            acc_y[i] = 6.0 * T2_inv *
                       (primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_y[i + 2] -
                        2 * primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_y[i + 1] +
                        primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_y[i]);
            acc_z[i] = 6.0 * T2_inv *
                       (primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_z[i + 2] -
                        2 * primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_z[i + 1] +
                        primitive_[tracker_idx][inter_visible_index_[tracker_idx][start_idx]].ctrl_z[i]);
        }
        for (int j = 0; j <= 2; j++) {
            value = 0.0;
            for (int k = std::max(0, j - 1); k <= std::min(1, j); k++) {
                value +=
                        double(dmvc3d::nchooser(1, k)) * double(dmvc3d::nchooser(1, j - k)) / double(dmvc3d::nchooser(2, j)) *
                        (acc_x[k] * acc_x[j - k] + acc_y[k] * acc_y[j - k] + acc_z[k] * acc_z[j - k]);
            }
            if (value > acc_max_squared) {
                flag_store_acceleration = false;
                break;
            }
        }
        if (flag_store_acceleration)
            dyn_feas_idx_sub.push_back(inter_visible_index_[tracker_idx][idx]);
    }
}

void dmvc3d::Tracker::GetSafeIndexThread(const int &tracker_idx, const int &start_idx, const int &end_idx,
                                         std::vector<uint> &safe_idx_sub) {
    bool flag_store_in1 = true; // collision between obstacle and tracker
    bool flag_store_in2 = true; // occlusion of targets
    bool flag_store_in3 = true; // distance between tracker and target
    bool flag_store_out = true;
    double value;
    double relative_obstacle_pos_x[4], relative_obstacle_pos_y[4], relative_obstacle_pos_z[4]; //drone-obstacle
    double relative_target_obstacle_pos_x[4], relative_target_obstacle_pos_y[4], relative_target_obstacle_pos_z[4]; //target-obstacle
    double relative_target_pos_x[4], relative_target_pos_y[4], relative_target_pos_z[4]; //drone-target
    double object_radius_squared = param_.object_radius * param_.object_radius;
    double safe_distance_squared = pow(param_.safe_distance + 2 * param_.object_radius, 2);
    double distance_max_squared = pow(param_.distance_max, 2);
    for (int idx = start_idx; idx < end_idx; idx++) {
        flag_store_in1 = true;
        flag_store_in2 = true;
        flag_store_out = true;
        flag_store_in3 = true;
        for (int j = 0; j < 4; j++) {
            relative_target_pos_x[j] = primitive_[tracker_idx][idx].ctrl_x[j] - target_trajectory_.ctrl_x[j];
            relative_target_pos_y[j] = primitive_[tracker_idx][idx].ctrl_y[j] - target_trajectory_.ctrl_y[j];
            relative_target_pos_z[j] = primitive_[tracker_idx][idx].ctrl_z[j] - target_trajectory_.ctrl_z[j];
        }
        if (obstacle_primitive_list_.empty())
            goto target_distance_check;
        for (int i = 0; i < obstacle_primitive_list_.size(); i++) {
            flag_store_out = true;
            for (int j = 0; j < 4; j++) {
                relative_obstacle_pos_x[j] =
                        primitive_[tracker_idx][idx].ctrl_x[j] - obstacle_primitive_list_[i].ctrl_x[j];
                relative_obstacle_pos_y[j] =
                        primitive_[tracker_idx][idx].ctrl_y[j] - obstacle_primitive_list_[i].ctrl_y[j];
                relative_obstacle_pos_z[j] =
                        primitive_[tracker_idx][idx].ctrl_z[j] - obstacle_primitive_list_[i].ctrl_z[j];
                relative_target_obstacle_pos_x[j] =
                        target_trajectory_.ctrl_x[j] - obstacle_primitive_list_[i].ctrl_x[j];
                relative_target_obstacle_pos_y[j] =
                        target_trajectory_.ctrl_y[j] - obstacle_primitive_list_[i].ctrl_y[j];
                relative_target_obstacle_pos_z[j] =
                        target_trajectory_.ctrl_z[j] - obstacle_primitive_list_[i].ctrl_z[j];
            }
            for (int j = 0; j <= 6; j++) {  // Collision between obstacle and tracker
                flag_store_in1 = true;
                value = 0.0f;
                for (int k = std::max(0, j - 3); k <= std::min(3, j); k++) {
                    value += (double) nchooser(3, k) * (double) nchooser(3, j - k) /
                             (double) nchooser(6, j) *
                             (relative_obstacle_pos_x[k] * relative_obstacle_pos_x[j - k] +
                              relative_obstacle_pos_y[k] * relative_obstacle_pos_y[j - k] +
                              relative_obstacle_pos_z[k] * relative_obstacle_pos_z[j - k]
                             );
                }
                if (value < safe_distance_squared) {
                    flag_store_in1 = false;
                    break;
                }
            }
            if (not flag_store_in1) {
                flag_store_out = false;
                break;
            }
            for (int j = 0; j <= 6; j++) {
                flag_store_in2 = true;
                value = 0.0f;
                for (int k = std::max(0, j - 3); k <= std::min(3, j); k++) {
                    value += (double) nchooser(3, k) * (double) nchooser(3, j - k) /
                             (double) nchooser(6, j) *
                             (relative_obstacle_pos_x[k] * relative_target_obstacle_pos_x[j - k] +
                              relative_obstacle_pos_y[k] * relative_target_obstacle_pos_y[j - k] +
                              relative_obstacle_pos_z[k] * relative_target_obstacle_pos_z[j - k]
                             );
                }
                if (value+2.0*object_radius_squared<0.0) {
                    flag_store_in2 = false;
                    break;
                }
            }
            if (not flag_store_in2) {
                flag_store_out = false;
                break;
            }
        }
        target_distance_check:
        {
            if (flag_store_in1 and flag_store_in2 and flag_store_out) {
                flag_store_in3 = true;
                for (int j = 0; j <= 6; j++) {
                    value = 0.0f;
                    for (int k = std::max(0, j - 3); k <= std::min(3, j); k++) {
                        value += (double) nchooser(3, k) * (double) nchooser(3, j - k) /
                                 (double) nchooser(6, j) *
                                 (relative_target_pos_x[k] * relative_target_pos_x[j - k] +
                                  relative_target_pos_y[k] * relative_target_pos_y[j - k]
                                 );
                    }
                    if (value < safe_distance_squared or value > distance_max_squared) {
                        flag_store_in3 = false;
                        break;
                    }
                }
            }
        };
        if (flag_store_out and flag_store_in3)
            safe_idx_sub.push_back(idx);
    }
}

void dmvc3d::Tracker::UpdateResultToBase(const bool &is_success) {
    {   // Success flag
        p_base_->mutex_set_[1].lock();
        p_base_->success_flag_ = is_success;
        p_base_->mutex_set_[1].unlock();
    }
    if (is_success) {
        {   // Primitive
            p_base_->mutex_set_[1].lock();
            p_base_->SetTrackerPrimitives(primitive_);
            p_base_->mutex_set_[1].unlock();
        }
        {   // Feasible Index
            p_base_->mutex_set_[1].lock();
            p_base_->SetFeasibleIndex(inter_safe_index_);
            p_base_->mutex_set_[1].unlock();
        }
        {   // Best Index
            vector<uint> best_index(num_tracker_);
            for (int i = 0; i < num_tracker_; i++)
                best_index[i] = dynamically_feasible_index_[i][0];

            p_base_->mutex_set_[1].lock();
            p_base_->SetBestIndex(best_index);
            p_base_->mutex_set_[1].unlock();
        }
        {   // Voronoi Cell
            p_base_->mutex_set_[1].lock();
            p_base_->SetMovingBufferedVoronoiCell(moving_buffered_voroni_cell_);
            p_base_->mutex_set_[1].unlock();
        }
//        {   // Visibility Cell
//            p_base_->mutex_set_[1].lock();
//            p_base_->SetVisibilityCell(visibility_cell_);
//            p_base_->mutex_set_[1].unlock();
//        }
        {
            p_base_->mutex_set_[1].lock();
            p_base_->SetCorridorVis(polys_);
            p_base_->mutex_set_[1].unlock();
        }
    } else {
        {   // Primitive
            p_base_->mutex_set_[1].lock();
            p_base_->EraseTrackerPrimitives();
            p_base_->mutex_set_[1].unlock();
        }
        {   // Safe Index
            p_base_->mutex_set_[1].lock();
            p_base_->EraseFeasibleIndex();
            p_base_->mutex_set_[1].unlock();
        }
        {   // Best Index
            p_base_->mutex_set_[1].lock();
            p_base_->EraseBestIndex();
            p_base_->mutex_set_[1].unlock();
        }
        {   // Moving Buffered Voronoi Cell
            p_base_->mutex_set_[1].lock();
            p_base_->EraseMovingBufferedVoronoiCell();
            p_base_->mutex_set_[1].unlock();
        }
        {
            p_base_->mutex_set_[1].lock();
            p_base_->EraseVisibilityCell();
            p_base_->mutex_set_[1].unlock();
        }
        {
            p_base_->mutex_set_[1].lock();
            p_base_->SetCorridorVis(polys_);
            p_base_->mutex_set_[1].unlock();
        }
    }
}

void dmvc3d::Tracker::GetInterSafetyIndexThread(const int &tracker_idx, const int &start_idx, const int &end_idx,
                                                std::vector<uint> &inter_safe_idx_sub) {
    bool flag_store_in = true;
    bool flag_store_out = true;
    vector<int> neighbor_drone_idx;
    for (int idx = 0; idx < current_tracker_state_.size(); idx++) {
        if (idx != tracker_idx)
            neighbor_drone_idx.push_back(idx);
    }
    int num_neighbor_drone = neighbor_drone_idx.size();
    vector<double> half_position(2);
    vector<double> relative_position(2);
    vector<vector<double>> half_position_list(num_neighbor_drone);
    vector<vector<double>> relative_position_list(num_neighbor_drone);
    vector<double> distance(num_neighbor_drone);

    for (int idx = 0; idx < num_neighbor_drone; idx++) {
        half_position[0] =
                0.5 * (current_tracker_state_[tracker_idx].px + current_tracker_state_[neighbor_drone_idx[idx]].px);
        half_position[1] =
                0.5 * (current_tracker_state_[tracker_idx].py + current_tracker_state_[neighbor_drone_idx[idx]].py);
        relative_position[0] =
                current_tracker_state_[neighbor_drone_idx[idx]].px - current_tracker_state_[tracker_idx].px;
        relative_position[1] =
                current_tracker_state_[neighbor_drone_idx[idx]].py - current_tracker_state_[tracker_idx].py;
        half_position_list[idx] = half_position;
        relative_position_list[idx] = relative_position;
        distance[idx] = sqrt(pow(relative_position[0], 2) + pow(relative_position[1], 2));
    }
    double relative_target_pos_x[4];
    double relative_target_pos_y[4];
    double value = 0.0;
    for (int primitive_idx = start_idx; primitive_idx < end_idx; primitive_idx++) {
        flag_store_out = true;
        for (int i = 0; i < 4; i++) {
            relative_target_pos_x[i] = primitive_[tracker_idx][safe_index_[tracker_idx][primitive_idx]].ctrl_x[i]
                                       - target_trajectory_.ctrl_x[i] + target_trajectory_.ctrl_x[0]; // Not Moving
            relative_target_pos_y[i] = primitive_[tracker_idx][safe_index_[tracker_idx][primitive_idx]].ctrl_y[i]
                                       - target_trajectory_.ctrl_y[i] + target_trajectory_.ctrl_y[0]; // Not Moving
        }
        for (int neighbor_idx = 0; neighbor_idx < num_neighbor_drone; neighbor_idx++) {
            flag_store_in = true;
            for (int i = 0; i < 4; i++) {
                value = relative_position_list[neighbor_idx][0] *
                        (half_position_list[neighbor_idx][0] - relative_target_pos_x[i]) +
                        relative_position_list[neighbor_idx][1] *
                        (half_position_list[neighbor_idx][1] - relative_target_pos_y[i]) -
                        param_.object_radius * distance[neighbor_idx];
                if (value < 0.0) {
                    flag_store_in = false;
                    break;
                }
            }
            if (not flag_store_in) {
                flag_store_out = false;
                break;
            }
        }
        if (flag_store_in and flag_store_out) {
            inter_safe_idx_sub.push_back(safe_index_[tracker_idx][primitive_idx]);
        }
    }
}

void dmvc3d::Tracker::GetBestIndex() {
    best_index_.clear();
    for (int idx = 0; idx < num_tracker_; idx++) {
        int num_chunk = dynamically_feasible_index_[idx].size() / param_.num_thread;
        vector<thread> worker_thread;
        vector<std::pair<dmvc3d::uint, double>> best_index_temp(param_.num_thread);
        dmvc3d::uint single_best_index;
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread.emplace_back(
                    thread(&Tracker::GetBestIndexThread, this, idx, num_chunk * j, num_chunk * (j + 1),
                           std::ref(best_index_temp[j])));
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread[j].join();
        double min_acceleration = 9999999.0;
        for (int j = 0; j < param_.num_thread; j++) {
            if (min_acceleration > best_index_temp[j].second) {
                min_acceleration = best_index_temp[j].second;
                single_best_index = best_index_temp[j].first;
            };
        }
        best_index_.push_back(single_best_index);
    }
}

void dmvc3d::Tracker::GetBestIndexThread(const int &tracker_idx, const int &start_idx, const int &end_idx,
                                         std::pair<uint, double> &score_pair) {
    double acc_coeff_x[2];
    double acc_coeff_y[2];
    double min_acc = 99999999.0;
    uint min_acc_idx = -1;
    double T = param_.horizon;
    double acc_coeff = 6.0 / pow(T, 2);
    double acc_squared_sum;
    for (int idx = start_idx; idx < end_idx; idx++) {
        acc_coeff_x[0] = acc_coeff * (primitive_[tracker_idx][dynamically_feasible_index_[tracker_idx][idx]].ctrl_x[2] -
                                      2.0 *
                                      primitive_[tracker_idx][dynamically_feasible_index_[tracker_idx][idx]].ctrl_x[1]
                                      +
                                      primitive_[tracker_idx][dynamically_feasible_index_[tracker_idx][idx]].ctrl_x[0]);
        acc_coeff_x[1] = acc_coeff * (primitive_[tracker_idx][dynamically_feasible_index_[tracker_idx][idx]].ctrl_x[3] -
                                      2.0 *
                                      primitive_[tracker_idx][dynamically_feasible_index_[tracker_idx][idx]].ctrl_x[2]
                                      +
                                      primitive_[tracker_idx][dynamically_feasible_index_[tracker_idx][idx]].ctrl_x[1]);
        acc_coeff_y[0] = acc_coeff * (primitive_[tracker_idx][dynamically_feasible_index_[tracker_idx][idx]].ctrl_y[2] -
                                      2.0 *
                                      primitive_[tracker_idx][dynamically_feasible_index_[tracker_idx][idx]].ctrl_y[1]
                                      +
                                      primitive_[tracker_idx][dynamically_feasible_index_[tracker_idx][idx]].ctrl_y[0]);
        acc_coeff_y[1] = acc_coeff * (primitive_[tracker_idx][dynamically_feasible_index_[tracker_idx][idx]].ctrl_y[3] -
                                      2.0 *
                                      primitive_[tracker_idx][dynamically_feasible_index_[tracker_idx][idx]].ctrl_y[2]
                                      +
                                      primitive_[tracker_idx][dynamically_feasible_index_[tracker_idx][idx]].ctrl_y[1]);
        acc_squared_sum = 0.0;
        for (int j = 0; j <= 2; j++) {
            for (int k = std::max(0, j - 1); k <= std::min(1, j); k++) {
                acc_squared_sum +=
                        double(dmvc3d::nchooser(1, k)) * double(dmvc3d::nchooser(1, j - k)) / double(dmvc3d::nchooser(2, j)) *
                        (acc_coeff_x[k] * acc_coeff_x[j - k] + acc_coeff_y[k] * acc_coeff_y[j - k]);
            }
        }
        if (acc_squared_sum < min_acc) {
            min_acc_idx = dynamically_feasible_index_[tracker_idx][idx];
            min_acc = acc_squared_sum;
        }
    }
    score_pair.second = min_acc;
    score_pair.first = min_acc_idx;
}

void dmvc3d::Tracker::CalculateMovingBufferedVoronoiCell() {
    moving_buffered_voroni_cell_.clear();
    moving_buffered_voroni_cell_.resize(num_tracker_);
    vector<AffineCoeff2D> single_moving_buffered_voronoi_cell;
    AffineCoeff2D half_space; //h1 x + h2 y + h3 <=0
    vector<dmvc3d::uint> neighbor_idx;
    for (int tracker_idx = 0; tracker_idx < num_tracker_; tracker_idx++) {
        neighbor_idx.clear();
        for (int i = 0; i < num_tracker_; i++) {
            if (i != tracker_idx)
                neighbor_idx.push_back(i);
        }
        single_moving_buffered_voronoi_cell.clear();
        for (int i = 0; i < neighbor_idx.size(); i++) {
            half_space[0] = current_tracker_state_[neighbor_idx[i]].px - current_tracker_state_[tracker_idx].px;
            half_space[1] = current_tracker_state_[neighbor_idx[i]].py - current_tracker_state_[tracker_idx].py;
            half_space[2] = param_.object_radius * sqrt(pow(
                    current_tracker_state_[neighbor_idx[i]].px - current_tracker_state_[tracker_idx].px, 2) +
                                                        pow(current_tracker_state_[neighbor_idx[i]].py -
                                                            current_tracker_state_[tracker_idx].py, 2)) +
                            0.5 * (current_tracker_state_[tracker_idx].px * current_tracker_state_[tracker_idx].px
                                   + current_tracker_state_[tracker_idx].py * current_tracker_state_[tracker_idx].py -
                                   current_tracker_state_[neighbor_idx[i]].px *
                                   current_tracker_state_[neighbor_idx[i]].px
                                   - current_tracker_state_[neighbor_idx[i]].py *
                                     current_tracker_state_[neighbor_idx[i]].py);
            single_moving_buffered_voronoi_cell.push_back(half_space);
        }
        moving_buffered_voroni_cell_[tracker_idx] = single_moving_buffered_voronoi_cell;
    }
}

void dmvc3d::Tracker::GetInterVisibleIndexThread(const int &tracker_idx, const int &start_idx, const int &end_idx,
                                                 std::vector<uint> &inter_visible_idx_sub) {
    bool flag_store_in = true;
    bool flag_store_out = true;
    double value;
    for (int primitive_idx = start_idx; primitive_idx < end_idx; primitive_idx++) {
        flag_store_out = true;
        for (int i = 0; i < visibility_cell_[tracker_idx].size(); i++) {
            flag_store_in = true;
            for (int j = 0; j < 4; j++) {
                value = visibility_cell_[tracker_idx][i][0] *
                        (primitive_[tracker_idx][inter_safe_index_[tracker_idx][primitive_idx]].ctrl_x[j]
                         - target_trajectory_.ctrl_x[j] + target_trajectory_.ctrl_x[0] // Not Moving
                        ) +
                        visibility_cell_[tracker_idx][i][1] *
                        (primitive_[tracker_idx][inter_safe_index_[tracker_idx][primitive_idx]].ctrl_y[j]
                         - target_trajectory_.ctrl_y[j] + target_trajectory_.ctrl_y[0] // Not Moving
                        ) +
                        visibility_cell_[tracker_idx][i][2];
                if (value > 0.0) {
                    flag_store_in = false;
                    break;
                }
            }
            if (not flag_store_in) {
                flag_store_out = false;
                break;
            }
        }
        if (flag_store_in and flag_store_out)
            inter_visible_idx_sub.push_back(inter_safe_index_[tracker_idx][primitive_idx]);
    }
}

void dmvc3d::Tracker::GetInterVisibleIndex() {
    CalculateVisibilityCell();
    inter_visible_index_.clear();
    for (int tracker_idx = 0; tracker_idx < num_tracker_; tracker_idx++) {
        int num_chunk = inter_safe_index_[tracker_idx].size() / param_.num_thread;
        vector<thread> worker_thread;
        vector<vector<dmvc3d::uint>> inter_visible_index_temp(param_.num_thread);
        vector<dmvc3d::uint> single_inter_visible_index;
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread.emplace_back(
                    thread(&Tracker::GetInterVisibleIndexThread, this, tracker_idx, num_chunk * j, num_chunk * (j + 1),
                           std::ref(inter_visible_index_temp[j])));
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread[j].join();
        for (int j = 0; j < param_.num_thread; j++) {
            for (int k = 0; k < inter_visible_index_temp[j].size(); k++)
                single_inter_visible_index.push_back(inter_visible_index_temp[j][k]);
        }
        inter_visible_index_.push_back(single_inter_visible_index);
    }
//    printf("TRACKER SIZE: %d\n",inter_visible_index_.size());
//    printf("0-th TRACKER INTER VISIBLE SIZE: %d\n",inter_visible_index_[0].size());
//    printf("1-th TRACKER INTER VISIBLE SIZE: %d\n",inter_visible_index_[1].size());
//    printf("2-th TRACKER INTER VISIBLE SIZE: %d\n",inter_visible_index_[2].size());

}

void dmvc3d::Tracker::CalculateVisibilityCell() {
    visibility_cell_.clear();
    visibility_cell_.resize(num_tracker_);
    vector<AffineCoeff2D> single_visibility_cell;
    vector<AffineCoeff2D> half_spaces; //h1 x + h2 y + h3 <=0
    vector<dmvc3d::uint> neighbor_idx;
    for (int tracker_idx = 0; tracker_idx < num_tracker_; tracker_idx++) {
        neighbor_idx.clear();
        for (int i = 0; i < num_tracker_; i++) {
            if (i != tracker_idx)
                neighbor_idx.push_back(i);
        }
        single_visibility_cell.clear();
        for (int i = 0; i < neighbor_idx.size(); i++) {
//            printf("TRACKER IDX: %d, NEIGHBOR IDX: %d \n",tracker_idx,neighbor_idx[i]);
            half_spaces.clear();
            half_spaces = CalculateSingleVisibilityCell(current_tracker_state_[tracker_idx],
                                                        current_tracker_state_[neighbor_idx[i]]);
//            printf("HALF SPACE SIZE: %d\n",half_spaces.size());
            for (int j = 0; j < 2; j++) {
                single_visibility_cell.push_back(half_spaces[j]);
            }
        }
//        printf("THE NUMBER OF SINGLE VISIBILITY HALF SPACE:%d\n",single_visibility_cell.size());
        visibility_cell_[tracker_idx] = single_visibility_cell;
    }
}

std::vector<dmvc3d::AffineCoeff2D>
dmvc3d::Tracker::CalculateSingleVisibilityCell(const dmvc3d::State &tracker_pos, const dmvc3d::State &neighbor_pos) {
    std::vector<dmvc3d::AffineCoeff2D> constraint;
    dmvc3d::AffineCoeff2D half_space;
    dmvc3d::Point v1{tracker_pos.px - target_trajectory_.ctrl_x[0],
                     tracker_pos.py - target_trajectory_.ctrl_y[0],
                     tracker_pos.pz - target_trajectory_.ctrl_z[0]};
    dmvc3d::Point v2{neighbor_pos.px - target_trajectory_.ctrl_x[0],
                     neighbor_pos.py - target_trajectory_.ctrl_y[0],
                     neighbor_pos.pz - target_trajectory_.ctrl_z[0]};

    double cross_product = v1.x * v2.y - v1.y * v2.x;
    dmvc3d::Point a{0.0, 0.0, 0.0};
    dmvc3d::Point b{0.0, 0.0, 0.0};
    if (cross_product > 0)
        a.x = v1.x, a.y = v1.y, b.x = v2.x, b.y = v2.y;
    else
        a.x = v2.x, a.y = v2.y, b.x = v1.x, b.y = v1.y;
    //
    double dot_product = a.x * b.x + a.y * b.y;
    double alpha_min = 1.0;
    double alpha_max;
    double alpha;
    double a_size = sqrt(pow(a.x, 2) + pow(a.y, 2));
    double b_size = sqrt(pow(b.x, 2) + pow(b.y, 2));
    double cos_value;
    double sin_value;
    if (dot_product < 0) {  // Obtuse Angle
        alpha_max = min(1 / param_.object_radius * min(a_size, b_size),
                        sqrt(2.0 / (1.0 - dot_product / (a_size * b_size))));
        alpha = 0.5 * (alpha_min + alpha_max);
        // First Constraint
        if (abs(v1.x - a.x) < 1e-4 and abs(v1.y - a.y) < 1e-4) {    // constraint for a
            half_space[0] = -a.x / a_size;
            half_space[1] = -a.y / a_size;
            half_space[2] = alpha * param_.object_radius;
            half_space[2] = half_space[2] - half_space[0] * target_trajectory_.ctrl_x[0] -
                            half_space[1] * target_trajectory_.ctrl_y[0];
        } else {
            half_space[0] = -b.x / b_size;
            half_space[1] = -b.y / b_size;
            half_space[2] = alpha * param_.object_radius;    //constraint for b
            half_space[2] = half_space[2] - half_space[0] * target_trajectory_.ctrl_x[0] -
                            half_space[1] * target_trajectory_.ctrl_y[0];
        }
        constraint.push_back(half_space);
        // Second Constraint
        if (abs(v1.x - a.x) < 1e-4 and abs(v1.y - a.y) < 1e-4) {    // constraint for a
            cos_value = b.x / b_size * sqrt(alpha * alpha - 1) / alpha + b.y / b_size / alpha;
            sin_value = b.y / b_size * sqrt(alpha * alpha - 1) / alpha - b.x / b_size / alpha;

            half_space[0] = -sin_value;
            half_space[1] = cos_value;
            half_space[2] = alpha * param_.object_radius * a.x / a_size * sin_value -
                            alpha * param_.object_radius * a.y / a_size * cos_value;
            half_space[2] = half_space[2] - half_space[0] * target_trajectory_.ctrl_x[0] -
                            half_space[1] * target_trajectory_.ctrl_y[0];
        } else { // constraint for b
            cos_value = a.x / a_size * sqrt(alpha * alpha - 1) / alpha - a.y / a_size / alpha;
            sin_value = a.y / a_size * sqrt(alpha * alpha - 1) / alpha + a.x / a_size / alpha;

            half_space[0] = sin_value;
            half_space[1] = -cos_value;
            half_space[2] = -alpha * param_.object_radius * b.x / b_size * sin_value +
                            alpha * param_.object_radius * b.y / b_size * cos_value;
            half_space[2] = half_space[2] - half_space[0] * target_trajectory_.ctrl_x[0] -
                            half_space[1] * target_trajectory_.ctrl_y[0];
        }
        constraint.push_back(half_space);
    } else {   //Acute Angle
        alpha_max = min(min(abs(cross_product) / param_.object_radius / a_size,
                            abs(cross_product) / param_.object_radius / b_size),
                        sqrt(2.0 * (1.0 + abs(dot_product) / a_size / b_size)));
        alpha = 0.5 * (alpha_min + alpha_max);
        // First Constraint
        if (abs(v1.x - a.x) < 1e-4 and abs(v1.y - a.y) < 1e-4) {    // constraint for a
            half_space[0] = -b.y;
            half_space[1] = b.x;
            half_space[2] = alpha * param_.object_radius * b_size;    //constraint for b
            half_space[2] = half_space[2] - half_space[0] * target_trajectory_.ctrl_x[0] -
                            half_space[1] * target_trajectory_.ctrl_y[0];
        } else {
            half_space[0] = a.y;
            half_space[1] = -a.x;
            half_space[2] = alpha * param_.object_radius * a_size;
            half_space[2] = half_space[2] - half_space[0] * target_trajectory_.ctrl_x[0] -
                            half_space[1] * target_trajectory_.ctrl_y[0];
        }
        constraint.push_back(half_space);
        // Second Cosntraint
        if (abs(v1.x - a.x) < 1e-4 and abs(v1.y - a.y) < 1e-4) {    // constraint for a
            cos_value = b.x / b_size *
                        sqrt(1.0 - 1.0 / pow(alpha, 2) * pow(cross_product, 2) / pow(a_size, 2) / pow(b_size, 2)) +
                        b.y / b_size * abs(cross_product) / alpha / a_size / b_size;
            sin_value = b.y / b_size *
                        sqrt(1.0 - 1.0 / pow(alpha, 2) * pow(cross_product, 2) / pow(a_size, 2) / pow(b_size, 2)) -
                        b.x / b_size * abs(cross_product) / alpha / a_size / b_size;

            half_space[0] = -sin_value;
            half_space[1] = cos_value;
            half_space[2] = alpha * param_.object_radius * b_size * a.x / abs(cross_product) * sin_value -
                            alpha * param_.object_radius * b_size * a.y / abs(cross_product) * cos_value;
            half_space[2] = half_space[2] - half_space[0] * target_trajectory_.ctrl_x[0] -
                            half_space[1] * target_trajectory_.ctrl_y[0];
        } else { // constraint for b
            cos_value = a.x / a_size *
                        sqrt(1.0 - 1.0 / pow(alpha, 2) * pow(cross_product, 2) / pow(a_size, 2) / pow(b_size, 2)) -
                        a.y / a_size * abs(cross_product) / alpha / a_size / b_size;
            sin_value = a.y / a_size *
                        sqrt(1.0 - 1.0 / pow(alpha, 2) * pow(cross_product, 2) / pow(a_size, 2) / pow(b_size, 2)) +
                        a.x / a_size * abs(cross_product) / alpha / a_size / b_size;

            half_space[0] = sin_value;
            half_space[1] = -cos_value;
            half_space[2] = -alpha * param_.object_radius * a_size * b.x / abs(cross_product) * sin_value +
                            alpha * param_.object_radius * a_size * b.y / abs(cross_product) * cos_value;
            half_space[2] = half_space[2] - half_space[0] * target_trajectory_.ctrl_x[0] -
                            half_space[1] * target_trajectory_.ctrl_y[0];
        }
        constraint.push_back(half_space);
    }
//    printf("ALPHA: %f\n",alpha);
    return constraint;
}

void dmvc3d::Tracker::GetSafeIndexUnstructuredThread(const int &tracker_idx, const int &start_idx, const int &end_idx,
                                                     std::vector<uint> &safe_idx_sub) {
//    printf("TRACKER NUMBER: %d\n",corridor_constraints_.size());
    Eigen::Vector3d A_comp_temp{0.0, 0.0, 0.0};
    double b_comp_temp(0.0);
    vector<Eigen::Vector3d> LinearConstraintA;
    vector<double> LinearConstraintb;
    int num_constraint = corridor_constraints_[tracker_idx].A().rows();
    int num_var = corridor_constraints_[tracker_idx].A().cols();

    for (int i = 0; i < num_constraint; i++) {
        A_comp_temp[0] = corridor_constraints_[tracker_idx].A().coeffRef(i, 0);
        A_comp_temp[1] = corridor_constraints_[tracker_idx].A().coeffRef(i, 1);
        A_comp_temp[2] = corridor_constraints_[tracker_idx].A().coeffRef(i, 2);
        b_comp_temp = corridor_constraints_[tracker_idx].b().coeffRef(i, 0);
        LinearConstraintA.push_back(A_comp_temp);
        LinearConstraintb.push_back(b_comp_temp);
    }
    bool flag_store_in1 = true; // collision between obstacle and tracker
    bool flag_store_out1 = true;
    bool flag_store_in2 = true; // distance between tracker and target
    double safe_distance_squared = pow(param_.safe_distance + 2 * param_.object_radius, 2);
    double distance_max_squared = pow(param_.distance_max, 2);
    double relative_target_pos_x[4], relative_target_pos_y[4]; // tracker-target
    double value_sfc;
    double value_distance;
    for (int idx = start_idx; idx < end_idx; idx++) {
        flag_store_out1 = true;
        flag_store_in1 = true;
        // SFC Constraint
        for (int i = 0; i < LinearConstraintA.size(); i++) {
            for (int j = 0; j < 4; j++) {
                value_sfc = LinearConstraintA[i][0] * (primitive_[tracker_idx][idx].ctrl_x[j]) +
                            LinearConstraintA[i][1] * (primitive_[tracker_idx][idx].ctrl_y[j]) +
                            LinearConstraintA[i][2] * (primitive_[tracker_idx][idx].ctrl_z[j]) +
                            - LinearConstraintb[i] + param_.object_radius + param_.safe_distance;
                if (value_sfc > 0.0) {
                    flag_store_in1 = false;
                    break;
                }
            }
            if (not flag_store_in1) {
                flag_store_out1 = false;
                break;
            }
        }

        // Distance Constraint
        if (flag_store_out1) {
            flag_store_in2 = true;
            for (int i = 0; i < 4; i++) {
                relative_target_pos_x[i] = primitive_[tracker_idx][idx].ctrl_x[i] - target_trajectory_.ctrl_x[i];
                relative_target_pos_y[i] = primitive_[tracker_idx][idx].ctrl_y[i] - target_trajectory_.ctrl_y[i];
            }
            for (int j = 0; j <= 6; j++) {
                value_distance = 0.0;
                for (int k = std::max(0, j - 3); k <= std::min(3, j); k++) {
                    value_distance += (double) dmvc3d::nchooser(3, k) * (double) dmvc3d::nchooser(3, j - k) /
                                      (double) dmvc3d::nchooser(6, j) *
                                      (relative_target_pos_x[k] * relative_target_pos_x[j - k] +
                                       relative_target_pos_y[k] * relative_target_pos_y[j - k]);
                }
                if (value_distance > distance_max_squared or value_distance < safe_distance_squared) {
                    flag_store_in2 = false;
                    break;
                }
            }
            if (flag_store_in2) {
                safe_idx_sub.push_back(idx);
            }
        }
    }
}

void dmvc3d::Tracker::GenerateCorridor() {
    polys_.clear();
    corridor_constraints_.clear();
    Eigen::Matrix<double, 3, 1> predicted_point;
    predicted_point[0] = target_trajectory_.ctrl_x[0];
    predicted_point[1] = target_trajectory_.ctrl_y[0];
    predicted_point[2] = target_trajectory_.ctrl_z[0];

    Eigen::Matrix<double, 3, 1> start_point;
    for (int idx = 0; idx < num_tracker_; idx++) {
        start_point[0] = current_tracker_state_[idx].px;
        start_point[1] = current_tracker_state_[idx].py;
        start_point[2] = current_tracker_state_[idx].pz;
        vec_Vec3f segment;
        segment.push_back(start_point), segment.push_back(predicted_point);
        EllipsoidDecomp3D decomp_util;
        decomp_util.set_obs(point_cloud_3d_);
        decomp_util.set_local_bbox(Vec3f(5.0, 5.0, 2.0));
        decomp_util.dilate(segment);
        vec_E<Polyhedron3D> polys;
        auto poly_hedrons = decomp_util.get_polyhedrons();
        polys_.push_back(poly_hedrons[0]);
        LinearConstraint3D corridor_constraint(0.5 * (start_point + predicted_point), poly_hedrons[0].hyperplanes());
        corridor_constraints_.push_back(corridor_constraint);
    }
}

void dmvc3d::Tracker::GetInterSafetyNonCooperativeIndex() {
    inter_safe_index_.clear();
    for (int tracker_idx = 0; tracker_idx < num_tracker_; tracker_idx++) {
        int num_chunk = safe_index_[tracker_idx].size() / param_.num_thread;
        vector<thread> worker_thread;
        vector<vector<dmvc3d::uint>> inter_safe_index_temp(param_.num_thread);
        vector<dmvc3d::uint> single_inter_safe_index;
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread.emplace_back(
                    thread(&Tracker::GetInterSafetyNonCooperativeIndexThread, this, tracker_idx, num_chunk * j,
                           num_chunk * (j + 1),
                           std::ref(inter_safe_index_temp[j])));
        for (int j = 0; j < param_.num_thread; j++)
            worker_thread[j].join();
        for (int j = 0; j < param_.num_thread; j++) {
            for (int k = 0; k < inter_safe_index_temp[j].size(); k++)
                single_inter_safe_index.push_back(inter_safe_index_temp[j][k]);
        }
        inter_safe_index_.push_back(single_inter_safe_index);
    }
}

void
dmvc3d::Tracker::GetInterSafetyNonCooperativeIndexThread(const int &tracker_idx, const int &start_idx, const int &end_idx,
                                                         std::vector<uint> &inter_safe_idx_sub) {
    vector<int> neighbor_drone_idx;
    for (int idx = 0; idx < current_tracker_state_.size(); idx++) {
        if (idx != tracker_idx)
            neighbor_drone_idx.push_back(idx);
    }
    double t0, tf;
    if (obstacle_primitive_list_.empty()) {
        t0 = 0.0, tf = param_.horizon;
    } else {
        t0 = obstacle_primitive_list_[0].t0, tf = obstacle_primitive_list_[0].tf;
    }

    std::vector<dmvc3d::PrimitivePlanning> neighbor_agent_primitive_list;
    for (int i = 0; i < neighbor_drone_idx.size(); i++) {
        dmvc3d::PrimitivePlanning temp_primitive{t0, tf};
        temp_primitive.ctrl_x[0] = current_tracker_state_[neighbor_drone_idx[i]].px;
        temp_primitive.ctrl_x[1] = current_tracker_state_[neighbor_drone_idx[i]].px +
                                   0.3333333 * current_tracker_state_[neighbor_drone_idx[i]].vx * param_.horizon;
        temp_primitive.ctrl_x[2] = current_tracker_state_[neighbor_drone_idx[i]].px +
                                   0.6666667 * current_tracker_state_[neighbor_drone_idx[i]].vx * param_.horizon;
        temp_primitive.ctrl_x[3] = current_tracker_state_[neighbor_drone_idx[i]].px +
                                   1.0000000 * current_tracker_state_[neighbor_drone_idx[i]].vx * param_.horizon;
        temp_primitive.ctrl_y[0] = current_tracker_state_[neighbor_drone_idx[i]].py;
        temp_primitive.ctrl_y[1] = current_tracker_state_[neighbor_drone_idx[i]].py +
                                   0.3333333 * current_tracker_state_[neighbor_drone_idx[i]].vy * param_.horizon;
        temp_primitive.ctrl_y[2] = current_tracker_state_[neighbor_drone_idx[i]].py +
                                   0.6666667 * current_tracker_state_[neighbor_drone_idx[i]].vy * param_.horizon;
        temp_primitive.ctrl_y[3] = current_tracker_state_[neighbor_drone_idx[i]].py +
                                   1.0000000 * current_tracker_state_[neighbor_drone_idx[i]].vy * param_.horizon;
        temp_primitive.ctrl_z[0] = current_tracker_state_[neighbor_drone_idx[i]].pz;
        temp_primitive.ctrl_z[1] = current_tracker_state_[neighbor_drone_idx[i]].pz +
                                   0.3333333 * current_tracker_state_[neighbor_drone_idx[i]].vz * param_.horizon;
        temp_primitive.ctrl_z[2] = current_tracker_state_[neighbor_drone_idx[i]].pz +
                                   0.6666667 * current_tracker_state_[neighbor_drone_idx[i]].vz * param_.horizon;
        temp_primitive.ctrl_z[3] = current_tracker_state_[neighbor_drone_idx[i]].pz +
                                   1.0000000 * current_tracker_state_[neighbor_drone_idx[i]].vz * param_.horizon;
        neighbor_agent_primitive_list.push_back(temp_primitive);
    }
    bool flag_store_in1 = true; // collision between neighbor and tracker
    bool flag_store_in2 = true; // occlusion of targets
    bool flag_store_in3 = true; // distance from target (dummy)
    bool flag_store_out = true;
    double value;
    double relative_neighbor_pos_x[4], relative_neighbor_pos_y[4]; //drone-obstacle
    double relative_target_neighbor_pos_x[4], relative_target_neighbor_pos_y[4]; //target-obstacle
    double relative_target_pos_x[4], relative_target_pos_y[4]; //drone-target
    double object_radius_squared = param_.object_radius * param_.object_radius;
    double safe_distance_squared = pow(param_.safe_distance + 2 * param_.object_radius, 2);
    double distance_max_squared = pow(param_.distance_max, 2);
    for (int idx = start_idx; idx < end_idx; idx++) {
        flag_store_in1 = true;
        flag_store_in2 = true;
        flag_store_in3 = true;
        flag_store_out = true;
        for (int j = 0; j < 4; j++) {
            relative_target_pos_x[j] = primitive_[tracker_idx][idx].ctrl_x[j] - target_trajectory_.ctrl_x[j];
            relative_target_pos_y[j] = primitive_[tracker_idx][idx].ctrl_y[j] - target_trajectory_.ctrl_y[j];
        }
        if (neighbor_agent_primitive_list.empty())
            goto target_distance_check;
        for (int i = 0; i < neighbor_agent_primitive_list.size(); i++) {
            flag_store_out = true;
            for (int j = 0; j < 4; j++) {
                relative_neighbor_pos_x[j] =
                        primitive_[tracker_idx][idx].ctrl_x[j] - neighbor_agent_primitive_list[i].ctrl_x[j];
                relative_neighbor_pos_y[j] =
                        primitive_[tracker_idx][idx].ctrl_y[j] - neighbor_agent_primitive_list[i].ctrl_y[j];
                relative_target_neighbor_pos_x[j] =
                        target_trajectory_.ctrl_x[j] - neighbor_agent_primitive_list[i].ctrl_x[j];
                relative_target_neighbor_pos_y[j] =
                        target_trajectory_.ctrl_y[j] - neighbor_agent_primitive_list[i].ctrl_y[j];
            }
            for (int j = 0; j <= 6; j++) {  // Collision between neighbor and tracker
                flag_store_in1 = true;
                value = 0.0f;
                for (int k = std::max(0, j - 3); k <= std::min(3, j); k++) {
                    value += (double) nchooser(3, k) * (double) nchooser(3, j - k) /
                             (double) nchooser(6, j) *
                             (relative_neighbor_pos_x[k] * relative_neighbor_pos_x[j - k] +
                              relative_neighbor_pos_y[k] * relative_neighbor_pos_y[j - k]
                             );
                }
                if (value < safe_distance_squared) {
                    flag_store_in1 = false;
                    break;
                }
            }
            if (not flag_store_in1) {
                flag_store_out = false;
                break;
            }
            for (int j = 0; j <= 6; j++) {
                flag_store_in2 = true;
                value = 0.0f;
                for (int k = std::max(0, j - 3); k <= std::min(3, j); k++) {
                    value += (double) nchooser(3, k) * (double) nchooser(3, j - k) /
                             (double) nchooser(6, j) *
                             (relative_neighbor_pos_x[k] * relative_target_neighbor_pos_x[j - k] +
                              relative_neighbor_pos_y[k] * relative_target_neighbor_pos_y[j - k]
                             );
                }
                if (value < object_radius_squared) {
                    flag_store_in2 = false;
                    break;
                }
            }
            if (not flag_store_in2) {
                flag_store_out = false;
                break;
            }
        }
        target_distance_check:
        {
            if (flag_store_in1 and flag_store_in2 and flag_store_out) {
                flag_store_in3 = true;
                for (int j = 0; j <= 6; j++) {
                    value = 0.0f;
                    for (int k = std::max(0, j - 3); k <= std::min(3, j); k++) {
                        value += (double) nchooser(3, k) * (double) nchooser(3, j - k) /
                                 (double) nchooser(6, j) *
                                 (relative_target_pos_x[k] * relative_target_pos_x[j - k] +
                                  relative_target_pos_y[k] * relative_target_pos_y[j - k]
                                 );
                    }
                    if (value < safe_distance_squared or value > distance_max_squared) {
                        flag_store_in3 = false;
                        break;
                    }
                }
            }
        };
        if (flag_store_out and flag_store_in3)
            inter_safe_idx_sub.push_back(idx);
    }
}
