//
// Created by larr-planning on 25. 1. 14.
//
#include <dmvc_replay3d/TotalTrajReader.h>

dmvc3d::TotalTrajReader::TotalTrajReader(): nh_("~") {
    nh_.param<int>("num_obstacle",num_obstacle_,0);
    nh_.param<int>("num_tracker",num_tracker_,0);
    nh_.param<int>("num_target",num_target_,0);
    nh_.param<std::string>("file_name",file_name_,"");
    nh_.param<std::string>("baseline_file_name",baseline_file_name_,"");
    nh_.param<std::string>("map_frame_id",map_frame_id_,"map");
    nh_.param<double>("inflation_size",inflation_size_,0.5);

    target_path_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("target_path_vis",1);
    tracker_path_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("tracker_path_vis",1);
    obstacle_path_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_path_vis",1);
    bearing_history_publisher_ = nh_.advertise<visualization_msgs::Marker>("bearing_vector_vis",1);

    target_path_baseline_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("target_path_baseline_vis",1);
    tracker_path_baseline_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("tracker_path_baseline_vis",1);
    bearing_history_baseline_publisher_ = nh_.advertise<visualization_msgs::Marker>("bearing_vector_baseline_vis",1);
    nh_.param<std::string>("obstacle_configuration_file_name", obstacle_configuration_file_name_, "");
    pcl_boxes_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("pcl_boxes", 1);

    ReadTotalTrajectory();
    ReadBaselineTotalTrajectory();
    UnstructuredObstacleConfiguration();
}

void dmvc3d::TotalTrajReader::Run() {
    ros::Rate loop_rate(3.0);
    while (ros::ok()){
        Publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void dmvc3d::TotalTrajReader::ReadTotalTrajectory() {
    std::vector<visualization_msgs::Marker> target_path_list;
    std::vector<visualization_msgs::Marker> obstacle_path_list;
    std::vector<visualization_msgs::Marker> tracker_path_list;
    target_path_list.resize(num_target_);
    obstacle_path_list.resize(num_obstacle_);
    tracker_path_list.resize(num_tracker_);

    for(int i=0;i<num_target_;i++){ // TARGET
        target_path_list[i].header.frame_id="map";
        target_path_list[i].color.a = 1.0;
        target_path_list[i].color.r = 1.0;
        target_path_list[i].color.g = 0.0;
        target_path_list[i].color.b = 0.0;
        target_path_list[i].action = visualization_msgs::Marker::ADD;
        target_path_list[i].scale.x = 0.05;
        target_path_list[i].id = i;
        target_path_list[i].ns = "TARGET_PATH_VIS";
        target_path_list[i].type = visualization_msgs::Marker::LINE_STRIP;
        target_path_list[i].pose.orientation.w = 1.0;
        target_path_list[i].pose.orientation.x = 0.0;
        target_path_list[i].pose.orientation.y = 0.0;
        target_path_list[i].pose.orientation.z = 0.0;
    }
    for(int i=0;i<num_tracker_;i++){ // TRACKER
        tracker_path_list[i].header.frame_id="map";
        tracker_path_list[i].color.a = 1.0;
        tracker_path_list[i].color.r = 0.0;
        tracker_path_list[i].color.g = 0.0;
        tracker_path_list[i].color.b = 1.0;
        tracker_path_list[i].action = visualization_msgs::Marker::ADD;
        tracker_path_list[i].scale.x = 0.05;
        tracker_path_list[i].id = i;
        tracker_path_list[i].ns = "TRACKER_PATH_VIS";
        tracker_path_list[i].type = visualization_msgs::Marker::LINE_STRIP;
        tracker_path_list[i].pose.orientation.w = 1.0;
        tracker_path_list[i].pose.orientation.x = 0.0;
        tracker_path_list[i].pose.orientation.y = 0.0;
        tracker_path_list[i].pose.orientation.z = 0.0;
    }
    for(int i=0;i<num_obstacle_;i++){ // TRACKER
        obstacle_path_list[i].header.frame_id="map";
        obstacle_path_list[i].color.a = 0.5;
        obstacle_path_list[i].color.r = 0.0;
        obstacle_path_list[i].color.g = 0.4;
        obstacle_path_list[i].color.b = 0.0;
        obstacle_path_list[i].action = visualization_msgs::Marker::ADD;
        obstacle_path_list[i].scale.x = 0.05;
        obstacle_path_list[i].id = i;
        obstacle_path_list[i].ns = "OBSTACLE_PATH_VIS";
        obstacle_path_list[i].type = visualization_msgs::Marker::LINE_STRIP;
        obstacle_path_list[i].pose.orientation.w = 1.0;
        obstacle_path_list[i].pose.orientation.x = 0.0;
        obstacle_path_list[i].pose.orientation.y = 0.0;
        obstacle_path_list[i].pose.orientation.z = 0.0;
    }
    bearing_list_vis_.header.frame_id="map";
    bearing_list_vis_.pose.orientation.w = 1.0;
    bearing_list_vis_.pose.orientation.x = 0.0;
    bearing_list_vis_.pose.orientation.y = 0.0;
    bearing_list_vis_.pose.orientation.z = 0.0;
    bearing_list_vis_.ns="BEARING_HISTORY_VIS";
    bearing_list_vis_.id=0;
    bearing_list_vis_.scale.x = 0.015;
    bearing_list_vis_.color.a = 0.15;
    bearing_list_vis_.color.r = 0.5;
    bearing_list_vis_.color.g = 0.0;
    bearing_list_vis_.color.b = 0.5;

    bearing_list_vis_.type=visualization_msgs::Marker::LINE_LIST;
    bearing_list_vis_.action=visualization_msgs::Marker::ADD;

    std::ifstream total_state_history_file;
    total_state_history_file.open(file_name_.c_str());
    double pos[3];
    double vel[3];
    double time;
    geometry_msgs::Point temp_target_point;
    geometry_msgs::Point temp_tracker_point;
    geometry_msgs::Point temp_obstacle_point;

    if(total_state_history_file.is_open()){
        std::string line;
        while(std::getline(total_state_history_file,line)){
            std::istringstream iss(line);
            iss>>time;
            for(int i=0;i<num_target_;i++){
                iss>>pos[0]>>pos[1]>>pos[2]>>vel[0]>>vel[1]>>vel[2];
                temp_target_point.x = pos[0], temp_target_point.y = pos[1], temp_target_point.z = pos[2];
                target_path_list[i].points.push_back(temp_target_point);
            }
            for(int i=0;i<num_tracker_;i++){
                iss>>pos[0]>>pos[1]>>pos[2]>>vel[0]>>vel[1]>>vel[2];
                temp_tracker_point.x = pos[0], temp_tracker_point.y = pos[1], temp_tracker_point.z = pos[2];
                tracker_path_list[i].points.push_back(temp_tracker_point);
                bearing_list_vis_.points.push_back(temp_target_point);
                bearing_list_vis_.points.push_back(temp_tracker_point);

            }
            for(int i=0;i<num_obstacle_;i++){
                iss>>pos[0]>>pos[1]>>pos[2]>>vel[0]>>vel[1]>>vel[2];
                temp_obstacle_point.x = pos[0], temp_obstacle_point.y = pos[1], temp_obstacle_point.z = pos[2];
                obstacle_path_list[i].points.push_back(temp_obstacle_point);
            }
        }
        for(int i =0;i<num_target_;i++)
            target_path_vis_.markers.push_back(target_path_list[i]);
        for(int i =0;i<num_tracker_;i++)
            tracker_path_list_vis_.markers.push_back(tracker_path_list[i]);
        for(int i=0;i<num_obstacle_;i++)
            obstacle_path_list_vis_.markers.push_back(obstacle_path_list[i]);


    }
    total_state_history_file.close();
}

void dmvc3d::TotalTrajReader::Publish() {
    // Baseline
//    if(not tracker_path_list_baseline_vis_.markers.empty()){
//        tracker_path_baseline_publisher_.publish(tracker_path_list_baseline_vis_);
////        bearing_history_baseline_publisher_.publish(bearing_list_baseline_vis_);
//    }
    if(not tracker_path_list_vis_.markers.empty())
        tracker_path_publisher_.publish(tracker_path_list_vis_);
    if(not obstacle_path_list_vis_.markers.empty())
        obstacle_path_publisher_.publish(obstacle_path_list_vis_);
    if(not (target_path_vis_.markers.empty() or tracker_path_list_vis_.markers.empty()))
        bearing_history_publisher_.publish(bearing_list_vis_);
    if(not target_path_vis_.markers.empty())
        target_path_publisher_.publish(target_path_vis_);
    if(not pcl_boxes_vis_.markers.empty())
        pcl_boxes_vis_publisher_.publish(pcl_boxes_vis_);
}

void dmvc3d::TotalTrajReader::ReadBaselineTotalTrajectory() {

}

void dmvc3d::TotalTrajReader::UnstructuredObstacleConfiguration() {
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
