//
// Created by larr-laptop on 25. 1. 12.
//
#include <dmvc_simulator3d/Simulator.h>

dmvc3d::Simulator::Simulator(): nh_("~") {
    nh_.param<string>("map_frame_id", map_frame_id_, "map");
    nh_.param<double>("simulation_dt", simulation_dt_, 0.01);

    nh_.param<bool>("is_unstructured", is_unstructured_, true);
    nh_.param<double>("inflation_size", inflation_size_, 0.5);
    nh_.param<double>("point_resolution", point_resolution_, 0.1);

    simulation_frequency_ = 1.0 / simulation_dt_;
    nh_.param<double>("agent_size", agent_size_, 0.1);

    // Total number of agents in files
    nh_.param<int>("total_object_number_in_file", total_object_number_in_file_, 0);

    // Obstacle Indices
    if (nh_.hasParam("obstacle_idx_list")) {
        nh_.getParam("obstacle_idx_list", object_idx_list_);
    } else
        ROS_ERROR("Failed to find 'obstacle list'.");

    // Target Index
    nh_.param<int>("target_idx", target_idx_, 0);
//    target_idx_ = 0; // Analysis
    object_idx_list_.push_back(target_idx_);
    cout<<"TARGET IDX: "<<target_idx_<<endl;
    object_number_ = (int) object_idx_list_.size(); // obstacles + target
    nh_.param<string>("initial_state_file_name", initial_state_file_name_, "");
    nh_.param<string>("object_history_file_name", object_history_file_name_, "");
    // unstructured file name
    nh_.param<string>("obstacle_configuration_file_name", obstacle_configuration_file_name_, "");
    nh_.param<int>("num_tracker", tracker_number_, 3);
    nh_.param<int>("moving_obstacle_number", moving_obstacle_number_, 0);
    // Analysis
//    nh_.param<int>("repetition", repetition_, 0);
//    nh_.param<int>("total_test_number", total_test_number_, 1);

    obstacle_vis_.header.frame_id = map_frame_id_;
    target_vis_.header.frame_id = map_frame_id_;

    tracker_vis_.header.frame_id = map_frame_id_;
    tracker_vis_.type = visualization_msgs::Marker::SPHERE;
    tracker_vis_.ns = "Tracker";
    tracker_vis_.color.a = 1.0;
    tracker_vis_.color.r = 0.0;
    tracker_vis_.color.g = 0.0;
    tracker_vis_.color.b = 1.0;
    tracker_vis_.scale.x = 2 * agent_size_;
    tracker_vis_.scale.y = 2 * agent_size_;
    tracker_vis_.scale.z = 2 * agent_size_;
    tracker_vis_.pose.orientation.w = 1.0;
    tracker_vis_.pose.orientation.x = 0.0;
    tracker_vis_.pose.orientation.y = 0.0;
    tracker_vis_.pose.orientation.z = 0.0;

    obstacle_vis_.type = visualization_msgs::Marker::SPHERE;
    obstacle_vis_.ns = "Obstacle";
    obstacle_vis_.color.a = 1.0;
    obstacle_vis_.color.r = 0.0;
    obstacle_vis_.color.g = 1.0;
    obstacle_vis_.color.b = 0.0;
    obstacle_vis_.scale.x = 2 * agent_size_;
    obstacle_vis_.scale.y = 2 * agent_size_;
    obstacle_vis_.scale.z = 2 * agent_size_;
    obstacle_vis_.pose.orientation.w = 1.0;
    obstacle_vis_.pose.orientation.x = 0.0;
    obstacle_vis_.pose.orientation.y = 0.0;
    obstacle_vis_.pose.orientation.z = 0.0;

    target_vis_.type = visualization_msgs::Marker::SPHERE;
    target_vis_.ns = "Target";
    target_vis_.id = 0;
    target_vis_.color.a = 1.0;
    target_vis_.color.r = 1.0;
    target_vis_.color.g = 0.0;
    target_vis_.color.b = 0.0;
    target_vis_.scale.x = 2 * agent_size_;
    target_vis_.scale.y = 2 * agent_size_;
    target_vis_.scale.z = 2 * agent_size_;
    target_vis_.pose.orientation.w = 1.0;
    target_vis_.pose.orientation.x = 0.0;
    target_vis_.pose.orientation.y = 0.0;
    target_vis_.pose.orientation.z = 0.0;

    point_cloud_.header.frame_id = map_frame_id_;
    point_cloud_range_.header.frame_id = map_frame_id_;

    target_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("target_vis", 1);
    obstacle_list_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_list_vis", 1);
    tracker_list_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("tracker_list_vis", 1);

    obstacle_state_list_publisher_ = nh_.advertise<dmvc_tracker3d::ObjectStateList>("obstacle_state_list", 1);
    target_state_publisher_ = nh_.advertise<dmvc_tracker3d::ObjectState>("target_state", 1);
    tracker_state_list_publisher_ = nh_.advertise<dmvc_tracker3d::ObjectStateList>("tracker_state_list", 1);

    pcl_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud_obstacle", 1);
    pcl_boxes_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("pcl_boxes", 1);


    control_input_subscriber_ = nh_.subscribe("/dmvc_tracker3d/tracker_control_input", 1,
                                              &Simulator::control_input_callback, this);
    ReadInitialTrackerStateList();
    ReadObjectTrajectory();
    if(is_unstructured_)
        ReadObstacleConfiguration();
}

void dmvc3d::Simulator::Run() {
    ros::Rate loop_rate(simulation_frequency_);
    double t0 = ros::Time::now().toSec();
    double t_sim;
    while(ros::ok()){
        t_sim = ros::Time::now().toSec()-t0;
        UpdateDynamics(t_sim);
        ProcessPointCloud();
        PrepareRosMsgs(t_sim);
        PublishRosMsgs();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void dmvc3d::Simulator::ReadInitialTrackerStateList() {
    std::ifstream initial_state_file;
    initial_state_file.open(initial_state_file_name_.c_str());
    State tracker_state;
    if (initial_state_file.is_open()) {
        while (initial_state_file >> tracker_state.px >> tracker_state.py >> tracker_state.pz >> tracker_state.vx
                                  >> tracker_state.vy >> tracker_state.vz) {
            current_tracker_state_.push_back(tracker_state);
        }
    }
    initial_state_file.close();
    tracker_number_ = (int) current_tracker_state_.size();
    for (int i = 0; i < tracker_number_; i++) {
        ControlInput zero_control_input{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        tracker_control_input.push_back(zero_control_input);
    }
}

void dmvc3d::Simulator::ReadObjectTrajectory() {
    ifstream object_trajectory_file;
    object_trajectory_file.open(object_history_file_name_.c_str());
    object_history_list_.clear();
    int num_read_unit = 12;
    string line, word;
    vector<string> row;
    vector<vector<string>> content;
    if (object_trajectory_file.is_open()) {
        while (getline(object_trajectory_file, line)) {
            row.clear();
            stringstream str(line);
            while (getline(str, word, ','))
                row.push_back(word);
            content.push_back(row);
        }
    } else
        printf("UNABLE TO READ OBJECT TRAJECTORY HISTORY\n");
    StateHistory temp_obstacle_history;
    for (int i = 0; i < object_number_; i++)
        object_history_list_.push_back(temp_obstacle_history);
    for (int i = 0; i < object_number_; i++) {
        for (int j = 0; j < content.size(); j++) {
            object_history_list_[i].t.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 1]));   // t
            object_history_list_[i].px.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 2]));   // px
            object_history_list_[i].py.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 3]));   // py
            object_history_list_[i].pz.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 4]));   // pz
            object_history_list_[i].vx.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 5]));   // vx
            object_history_list_[i].vy.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 6]));   // vy
            object_history_list_[i].vz.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 7]));   // vz
        }
    }
}

void dmvc3d::Simulator::ReadObstacleConfiguration() {
    std::ifstream obstacle_file;
    obstacle_file.open(obstacle_configuration_file_name_.c_str());
    double v1[3]{0.0, 0.0, 0.0};
    double v2[3]{0.0, 0.0, 0.0};
    double min_x, min_y, min_z, max_x, max_y, max_z;
    double p1[3], p2[3], p3[3], p4[3], p5[3], p6[3], p7[3], p8[3];
    int num_point_x;
    int num_point_y;
    int num_point_z;
    /*
     * p7--------p8
     *
     * p5---------p6
     * p3--------p4
     *
     * p1---------p2
    */
    pcl::PointXYZ temp_point;

    visualization_msgs::Marker single_obstacle;
    single_obstacle.pose.orientation.w = 1.0;
    single_obstacle.pose.orientation.x = 0.0;
    single_obstacle.pose.orientation.y = 0.0;
    single_obstacle.pose.orientation.z = 0.0;
    single_obstacle.color.a = 0.5;
    single_obstacle.color.r = 0.0;
    single_obstacle.color.g = 0.0;
    single_obstacle.color.b = 0.0;
    single_obstacle.ns = "PCL_OBSTACLE";
    single_obstacle.header.frame_id = map_frame_id_;
    single_obstacle.type = visualization_msgs::Marker::CUBE;
    int id = 0;
    if (obstacle_file.is_open()) {
        while (obstacle_file >> v1[0] >> v1[1] >> v1[2] >> v2[0] >> v2[1] >> v2[2]) {
            if (v1[0] <= v2[0]) {
                min_x = v1[0];
                max_x = v2[0];
            } else {
                min_x = v2[0];
                max_x = v1[0];
            }
            if (v1[1] <= v2[1]) {
                min_y = v1[1];
                max_y = v2[1];
            } else {
                min_y = v2[1];
                max_y = v1[1];
            }
            if (v1[2] <= v2[2]) {
                min_z = v1[2];
                max_z = v2[2];
            } else {
                min_z = v2[2];
                max_z = v1[2];
            }
            p1[0] = min_x - inflation_size_;
            p1[1] = min_y - inflation_size_;
            p1[2] = min_z - inflation_size_;

            p2[0] = max_x + inflation_size_;
            p2[1] = min_y - inflation_size_;
            p2[2] = min_z - inflation_size_;

            p3[0] = min_x - inflation_size_;
            p3[1] = max_y + inflation_size_;
            p3[2] = min_z - inflation_size_;

            p4[0] = max_x + inflation_size_;
            p4[1] = max_y + inflation_size_;
            p4[2] = min_z - inflation_size_;

            p5[0] = min_x - inflation_size_;
            p5[1] = min_y - inflation_size_;
            p5[2] = max_z + inflation_size_;

            p6[0] = max_x + inflation_size_;
            p6[1] = min_y - inflation_size_;
            p6[2] = max_z + inflation_size_;

            p7[0] = min_x - inflation_size_;
            p7[1] = max_y + inflation_size_;
            p7[2] = max_z + inflation_size_;

            p8[0] = max_x + inflation_size_;
            p8[1] = max_y + inflation_size_;
            p8[2] = max_z + inflation_size_;

            // P1 P2 P3 P4
            num_point_x = floor((p2[0]-p1[0])/point_resolution_+0.5);
            num_point_y = floor ( (p4[1]-p2[1])/point_resolution_+0.5);
            for(int i =0;i<num_point_x;i++){
                for(int j =0;j<num_point_y;j++){
                    temp_point.x = float(p1[0]+(p2[0]-p1[0])/num_point_x*i);
                    temp_point.y = float(p2[1]+(p4[1]-p2[1])/num_point_y*j);
                    temp_point.z = float(p1[2]);
                    point_cloud_.push_back(temp_point);
                }
            }
            // P5 P6 P7 P8
            num_point_x = floor((p6[0]-p5[0])/point_resolution_+0.5);
            num_point_y = floor ( (p8[1]-p6[1])/point_resolution_+0.5);
            for(int i =0;i<num_point_x;i++){
                for(int j =0;j<num_point_y;j++){
                    temp_point.x = float(p5[0]+(p6[0]-p5[0])/num_point_x*i);
                    temp_point.y = float(p6[1]+(p8[1]-p6[1])/num_point_y*j);
                    temp_point.z = float(p5[2]);
                    point_cloud_.push_back(temp_point);
                }
            }
            // P1 P2 P5 P6
            num_point_x = floor((p2[0]-p1[0])/point_resolution_+0.5);
            num_point_z = floor ( (p6[2]-p2[2])/point_resolution_+0.5);
            for(int i =0;i<num_point_x;i++){
                for(int k =0;k<num_point_z;k++){
                    temp_point.x = float(p1[0]+(p2[0]-p1[0])/num_point_x*i);
                    temp_point.y = float(p1[1]);
                    temp_point.z = float(p2[2]+(p6[2]-p2[2])/num_point_z*k);
                    point_cloud_.push_back(temp_point);
                }
            }
            // P3 P4 P7 P8
            num_point_x = floor((p4[0]-p3[0])/point_resolution_+0.5);
            num_point_z = floor ( (p8[2]-p4[2])/point_resolution_+0.5);
            for(int i =0;i<num_point_x;i++){
                for(int k =0;k<num_point_z;k++){
                    temp_point.x = float(p3[0]+(p4[0]-p3[0])/num_point_x*i);
                    temp_point.y = float(p3[1]);
                    temp_point.z = float(p4[2]+(p8[2]-p4[2])/num_point_z*k);
                    point_cloud_.push_back(temp_point);
                }
            }
            // P1 P3 P5 P7
            num_point_y = floor((p3[1]-p1[1])/point_resolution_+0.5);
            num_point_z = floor ( (p7[2]-p3[2])/point_resolution_+0.5);
            for(int j =0;j<num_point_y;j++){
                for(int k =0;k<num_point_z;k++){
                    temp_point.x = float(p1[0]);
                    temp_point.y = float(p1[1]+(p3[1]-p1[1])/num_point_y*j);
                    temp_point.z = float(p3[2]+(p7[2]-p3[2])/num_point_z*k);
                    point_cloud_.push_back(temp_point);
                }
            }
            // P2 P4 P6 P8
            num_point_y = floor((p4[1]-p2[1])/point_resolution_+0.5);
            num_point_z = floor ( (p8[2]-p4[2])/point_resolution_+0.5);
            for(int j =0;j<num_point_y;j++){
                for(int k =0;k<num_point_z;k++){
                    temp_point.x = float(p2[0]);
                    temp_point.y = float(p2[1]+(p4[1]-p2[1])/num_point_y*j);
                    temp_point.z = float(p4[2]+(p8[2]-p4[2])/num_point_z*k);
                    point_cloud_.push_back(temp_point);
                }
            }

            single_obstacle.id = id++;
            single_obstacle.pose.position.x = 0.125 * (p1[0] + p2[0] + p3[0] + p4[0]+p5[0] + p6[0] + p7[0] + p8[0]);
            single_obstacle.pose.position.y = 0.125 * (p1[1] + p2[1] + p3[1] + p4[1]+p5[1] + p6[1] + p7[1] + p8[1]);
            single_obstacle.pose.position.z = 0.125 * (p1[2] + p2[2] + p3[2] + p4[2]+p5[2] + p6[2] + p7[2] + p8[2]);
            single_obstacle.scale.x = p2[0] - p1[0];
            single_obstacle.scale.y = p3[1] - p1[1];
            single_obstacle.scale.z = p5[2] - p1[2];
            pcl_boxes_vis_.markers.push_back(single_obstacle);
        }
    }
    obstacle_file.close();
}

void dmvc3d::Simulator::PrepareRosMsgs(const double &t) {
    // Target Visualization
    target_vis_.pose.position.x = current_target_state_.px;
    target_vis_.pose.position.y = current_target_state_.py;
    target_vis_.pose.position.z = current_target_state_.pz;
    // Obstacle Visualization
    obstacle_list_vis_.markers.clear();
    for (int i = 0; i < current_obstacle_state_list_.size(); i++) {
        obstacle_vis_.id = i;
        obstacle_vis_.ns = std::to_string(i);
        obstacle_vis_.pose.position.x = current_obstacle_state_list_[i].px;
        obstacle_vis_.pose.position.y = current_obstacle_state_list_[i].py;
        obstacle_vis_.pose.position.z = current_obstacle_state_list_[i].pz;
        obstacle_list_vis_.markers.push_back(obstacle_vis_);
    }
    // Tracker Visualization
    tracker_list_vis_.markers.clear();
    for (int i = 0; i < current_tracker_state_.size(); i++) {
        tracker_vis_.id = i;
        tracker_vis_.pose.position.x = current_tracker_state_[i].px;
        tracker_vis_.pose.position.y = current_tracker_state_[i].py;
        tracker_vis_.pose.position.z = current_tracker_state_[i].pz;
        tracker_list_vis_.markers.push_back(tracker_vis_);
    }
    // target_state
    target_state_msg_.px = current_target_state_.px;
    target_state_msg_.py = current_target_state_.py;
    target_state_msg_.pz = current_target_state_.pz;
    target_state_msg_.vx = current_target_state_.vx;
    target_state_msg_.vy = current_target_state_.vy;
    target_state_msg_.vz = current_target_state_.vz;
    // obstacle_state
    dmvc_tracker3d::ObjectState object_state_temp;
    obstacle_state_list_msg_.object_state_list.clear();
    for (int i = 0; i < current_obstacle_state_list_.size(); i++) {
        object_state_temp.px = current_obstacle_state_list_[i].px;
        object_state_temp.py = current_obstacle_state_list_[i].py;
        object_state_temp.pz = current_obstacle_state_list_[i].pz;
        object_state_temp.vx = current_obstacle_state_list_[i].vx;
        object_state_temp.vy = current_obstacle_state_list_[i].vy;
        object_state_temp.vz = current_obstacle_state_list_[i].vz;
        obstacle_state_list_msg_.object_state_list.push_back(object_state_temp);
    }
    tracker_state_list_msg_.object_state_list.clear();
    for (int i = 0; i < current_tracker_state_.size(); i++) {
        object_state_temp.px = current_tracker_state_[i].px;
        object_state_temp.py = current_tracker_state_[i].py;
        object_state_temp.pz = current_tracker_state_[i].pz;
        object_state_temp.vx = current_tracker_state_[i].vx;
        object_state_temp.vy = current_tracker_state_[i].vy;
        object_state_temp.vz = current_tracker_state_[i].vz;
        tracker_state_list_msg_.object_state_list.push_back(object_state_temp);
    }
}

void dmvc3d::Simulator::PublishRosMsgs() {
    if(not pcl_boxes_vis_.markers.empty())
        pcl_boxes_vis_publisher_.publish(pcl_boxes_vis_);
    if(not point_cloud_range_.points.empty())
        pcl_publisher_.publish(point_cloud_range_);
    tracker_list_vis_publisher_.publish(tracker_list_vis_);
    obstacle_list_vis_publisher_.publish(obstacle_list_vis_);
    target_vis_publisher_.publish(target_vis_);
    tracker_state_list_publisher_.publish(tracker_state_list_msg_);
    obstacle_state_list_publisher_.publish(obstacle_state_list_msg_);
    target_state_publisher_.publish(target_state_msg_);
}

void dmvc3d::Simulator::UpdateDynamics(const double &t) {
    current_obstacle_state_list_.clear();
    State temp_state;
    for (int i = 0; i < object_number_; i++) {
        if(object_idx_list_[i]==target_idx_){
            current_target_state_.px = dmvc3d::interpolate(object_history_list_[i].t,
                                                         object_history_list_[i].px, t);
            current_target_state_.py = dmvc3d::interpolate(object_history_list_[i].t,
                                                         object_history_list_[i].py, t);
            current_target_state_.pz = dmvc3d::interpolate(object_history_list_[i].t,
                                                         object_history_list_[i].pz, t);
            current_target_state_.vx = dmvc3d::interpolate(object_history_list_[i].t,
                                                         object_history_list_[i].vx, t);
            current_target_state_.vy = dmvc3d::interpolate(object_history_list_[i].t,
                                                         object_history_list_[i].vy, t);
            current_target_state_.vz = dmvc3d::interpolate(object_history_list_[i].t,
                                                         object_history_list_[i].vz, t);
//            cout<<"PX: "<<current_target_state_.px<<", PY: "<<current_target_state_.py<<", PZ: "<<current_target_state_.pz<<endl;
        }
        else{
            temp_state.px = dmvc3d::interpolate(object_history_list_[i].t, object_history_list_[i].px, t);
            temp_state.py = dmvc3d::interpolate(object_history_list_[i].t, object_history_list_[i].py, t);
            temp_state.pz = dmvc3d::interpolate(object_history_list_[i].t, object_history_list_[i].pz, t);
            temp_state.vx = dmvc3d::interpolate(object_history_list_[i].t, object_history_list_[i].vx, t);
            temp_state.vy = dmvc3d::interpolate(object_history_list_[i].t, object_history_list_[i].vy, t);
            temp_state.vz = dmvc3d::interpolate(object_history_list_[i].t, object_history_list_[i].vz, t);
            current_obstacle_state_list_.push_back(temp_state);
        }
    }
    static double t_prev = t;
    double dt = t - t_prev;
    for (int i = 0; i < tracker_number_; i++) {
        current_tracker_state_[i].px = current_tracker_state_[i].px + current_tracker_state_[i].vx * dt;
        current_tracker_state_[i].py = current_tracker_state_[i].py + current_tracker_state_[i].vy * dt;
        current_tracker_state_[i].pz = current_tracker_state_[i].pz + current_tracker_state_[i].vz * dt;
        current_tracker_state_[i].vx = current_tracker_state_[i].vx + tracker_control_input[i].ax * dt;
        current_tracker_state_[i].vy = current_tracker_state_[i].vy + tracker_control_input[i].ay * dt;
        current_tracker_state_[i].vz = current_tracker_state_[i].vz + tracker_control_input[i].az * dt;
    }
    t_prev = t;

}

void dmvc3d::Simulator::ShuffleScenario() {

}

void dmvc3d::Simulator::control_input_callback(const dmvc_tracker3d::ControlInputList &msg) {
    for (int i = 0; i < tracker_number_; i++) {
        tracker_control_input[i].px = msg.control_input_list[i].px;
        tracker_control_input[i].py = msg.control_input_list[i].py;
        tracker_control_input[i].pz = msg.control_input_list[i].pz;
        tracker_control_input[i].vx = msg.control_input_list[i].vx;
        tracker_control_input[i].vy = msg.control_input_list[i].vy;
        tracker_control_input[i].vz = msg.control_input_list[i].vz;
        tracker_control_input[i].ax = msg.control_input_list[i].ax;
        tracker_control_input[i].ay = msg.control_input_list[i].ay;
        tracker_control_input[i].az = msg.control_input_list[i].az;
    }
}

void dmvc3d::Simulator::ProcessPointCloud() {
    point_cloud_range_.points.clear();
    for(int i=0;i<point_cloud_.points.size();i++){
        if(abs(point_cloud_.points[i].x-current_target_state_.px)< 5.0 and abs(point_cloud_.points[i].y-current_target_state_.py)< 5.0)
            point_cloud_range_.points.push_back(point_cloud_.points[i]);
    }
}
