//
// Created by larr-planning on 25. 1. 15.
//
#include <dmvc_replay3d/Replayer.h>

dmvc3d::Replayer::Replayer() : nh_("~") {
    nh_.param<std::string>("map_frame_id", param_.map_frame_id, "map");
    nh_.param<std::string>("state_history_file_name", state_history_file_name_, "");
    nh_.param<bool>("is_dynamic_experiment", is_dynamic_experiment_, false);
    nh_.param<bool>("do_logging",do_logging_,false);

    tracker_list_subscriber_ = nh_.subscribe("/dmvc_simulator3d/tracker_state_list", 1, &Replayer::TrackerListCallback,
                                             this);
    target_subscriber_ = nh_.subscribe("/dmvc_simulator3d/target_state", 1, &Replayer::TargetCallback, this);
    dynamic_obstacle_subscriber_ = nh_.subscribe("/dmvc_simulator3d/obstacle_state_list", 1,
                                                 &Replayer::DynamicObstacleCallback, this);

    tracker_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/dmvc_simulator3d/tracker_list_vis", 1);
    target_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("/dmvc_simulator3d/target_vis", 1);
    dynamic_obstacle_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/dmvc_simulator3d/obstacle_list_vis", 1);
    bearing_vector_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("bearing_vector_vis", 1);
    t0_ = ros::Time::now().toSec();

}

void dmvc3d::Replayer::Run() {
    ros::Rate loop_rate(40.0);
    while (ros::ok()) {
        PublishMsgs();
        WriteStateHistory();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void dmvc3d::Replayer::TrackerListCallback(const dmvc_tracker3d::ObjectStateList &tracker_list) {
    tracker_list_.clear();
    State temp_state;
    for (int idx = 0; idx < tracker_list.object_state_list.size(); idx++) {
        temp_state.px = tracker_list.object_state_list[idx].px;
        temp_state.py = tracker_list.object_state_list[idx].py;
        temp_state.pz = tracker_list.object_state_list[idx].pz;
        temp_state.vx = tracker_list.object_state_list[idx].vx;
        temp_state.vy = tracker_list.object_state_list[idx].vy;
        temp_state.vz = tracker_list.object_state_list[idx].vz;
        tracker_list_.push_back(temp_state);
    }
    tracker_info_flag_ = true;
}

void dmvc3d::Replayer::TargetCallback(const dmvc_tracker3d::ObjectState &target) {
    target_.px = target.px;
    target_.py = target.py;
    target_.pz = target.pz;
    target_.vx = target.vx;
    target_.vy = target.vy;
    target_.vz = target.vz;
    target_info_flag_ = true;
}

void dmvc3d::Replayer::DynamicObstacleCallback(const dmvc_tracker3d::ObjectStateList &obstacle_list) {
    dynamic_obstacle_list_.clear();
    State temp_state;
    for (int idx = 0; idx < obstacle_list.object_state_list.size(); idx++) {
        temp_state.px = obstacle_list.object_state_list[idx].px;
        temp_state.py = obstacle_list.object_state_list[idx].py;
        temp_state.pz = obstacle_list.object_state_list[idx].pz;
        temp_state.vx = obstacle_list.object_state_list[idx].vx;
        temp_state.vy = obstacle_list.object_state_list[idx].vy;
        temp_state.vz = obstacle_list.object_state_list[idx].vz;
        dynamic_obstacle_list_.push_back(temp_state);
    }
    dynamic_obstacle_info_flag = true;
}

void dmvc3d::Replayer::PublishMsgs() {
    if (tracker_info_flag_) {   // Tracker Visualization
        visualization_msgs::Marker single_tracker;
        visualization_msgs::MarkerArray tracker_list_vis;
        single_tracker.header.frame_id = "map";
        single_tracker.type = visualization_msgs::Marker::SPHERE;
        single_tracker.scale.x = 0.075 * 2;
        single_tracker.scale.y = 0.075 * 2;
        single_tracker.scale.z = 0.075 * 2;
        single_tracker.color.a = 1.0;
        single_tracker.color.r = 0.0;
        single_tracker.color.g = 0.0;
        single_tracker.color.b = 1.0;
        single_tracker.ns = "Tracker";
        single_tracker.pose.orientation.w = 1.0;
        single_tracker.pose.orientation.x = 0.0;
        single_tracker.pose.orientation.y = 0.0;
        single_tracker.pose.orientation.z = 0.0;
        for (int idx = 0; idx < tracker_list_.size(); idx++) {
            single_tracker.id = idx;
            single_tracker.pose.position.x = tracker_list_[idx].px;
            single_tracker.pose.position.y = tracker_list_[idx].py;
            single_tracker.pose.position.z = tracker_list_[idx].pz;
            tracker_list_vis.markers.push_back(single_tracker);
        }
        tracker_vis_publisher_.publish(tracker_list_vis);
    }
    if (target_info_flag_) {   // Target Visualization
        visualization_msgs::Marker single_target;
        single_target.header.frame_id = "map";
        single_target.type = visualization_msgs::Marker::SPHERE;
        single_target.scale.x = 0.075 * 2;
        single_target.scale.y = 0.075 * 2;
        single_target.scale.z = 0.075 * 2;
        single_target.color.a = 1.0;
        single_target.color.r = 1.0;
        single_target.color.g = 0.0;
        single_target.color.b = 0.0;
        single_target.ns = "Target";
        single_target.id = 0;
        single_target.pose.orientation.w = 1.0;
        single_target.pose.orientation.x = 0.0;
        single_target.pose.orientation.y = 0.0;
        single_target.pose.orientation.z = 0.0;
        single_target.pose.position.x = target_.px;
        single_target.pose.position.y = target_.py;
        single_target.pose.position.z = target_.pz;
        target_vis_publisher_.publish(single_target);
    }
    if (dynamic_obstacle_info_flag) {   // Dynamic Obstacle Visualization
        visualization_msgs::Marker single_obstacle;
        visualization_msgs::MarkerArray obstacle_list_vis;
        single_obstacle.header.frame_id = "map";
        single_obstacle.type = visualization_msgs::Marker::SPHERE;
        single_obstacle.scale.x = 0.075 * 2;
        single_obstacle.scale.y = 0.075 * 2;
        single_obstacle.scale.z = 0.075 * 2;
        single_obstacle.color.a = 1.0;
        single_obstacle.color.r = 0.0;
        single_obstacle.color.g = 1.0;
        single_obstacle.color.b = 0.0;
        single_obstacle.ns = "Obstacle";
        single_obstacle.pose.orientation.w = 1.0;
        single_obstacle.pose.orientation.x = 0.0;
        single_obstacle.pose.orientation.y = 0.0;
        single_obstacle.pose.orientation.z = 0.0;
        for (int idx = 0; idx < dynamic_obstacle_list_.size(); idx++) {
            single_obstacle.id = idx;
            single_obstacle.pose.position.x = dynamic_obstacle_list_[idx].px;
            single_obstacle.pose.position.y = dynamic_obstacle_list_[idx].py;
            single_obstacle.pose.position.z = dynamic_obstacle_list_[idx].pz;
            obstacle_list_vis.markers.push_back(single_obstacle);
        }
        dynamic_obstacle_vis_publisher_.publish(obstacle_list_vis);
    }
    if (tracker_info_flag_ and target_info_flag_) {   // Bearing Vector Visualization
        visualization_msgs::Marker bearing_list;
        bearing_list.header.frame_id = "map";
        bearing_list.type = visualization_msgs::Marker::LINE_LIST;
        bearing_list.color.a = 1.0;
        bearing_list.color.r = 0.5;
        bearing_list.color.g = 0.0;
        bearing_list.color.b = 0.5;
        bearing_list.scale.x = 0.025;
        bearing_list.pose.orientation.w = 1.0;
        bearing_list.pose.orientation.x = 0.0;
        bearing_list.pose.orientation.y = 0.0;
        bearing_list.pose.orientation.z = 0.0;

        geometry_msgs::Point tracker_position;
        geometry_msgs::Point target_position;
        target_position.x = target_.px;
        target_position.y = target_.py;
        target_position.z = target_.pz;
        for (int idx = 0; idx < tracker_list_.size(); idx++) {
            tracker_position.x = tracker_list_[idx].px;
            tracker_position.y = tracker_list_[idx].py;
            tracker_position.z = tracker_list_[idx].pz;
            bearing_list.points.push_back(target_position);
            bearing_list.points.push_back(tracker_position);
        }
        bearing_vector_vis_publisher_.publish(bearing_list);
    }

}

void dmvc3d::Replayer::WriteStateHistory() {
    if(not do_logging_)
        return;

    double time = GetCurrentTime();
    if (is_dynamic_experiment_) { // dynamic obstacle scenario
        if (dynamic_obstacle_info_flag and tracker_info_flag_ and target_info_flag_) {
            std::ofstream outfile;
            outfile.open(state_history_file_name_, std::ios_base::app);
            outfile << time << " " << target_.px << " " << target_.py << " " << target_.pz << " " << target_.vx << " "
                    << target_.vy << " " << target_.vz << " ";

            for (int i = 0; i < tracker_list_.size(); i++) {
                outfile << tracker_list_[i].px << " " << tracker_list_[i].py << " " << tracker_list_[i].pz << " "
                        << tracker_list_[i].vx << " " << tracker_list_[i].vy << " " << tracker_list_[i].vz << " ";
            }
            for (int i = 0; i < dynamic_obstacle_list_.size(); i++) {
                outfile << dynamic_obstacle_list_[i].px << " " << dynamic_obstacle_list_[i].py << " "
                        << dynamic_obstacle_list_[i].pz << " "
                        << dynamic_obstacle_list_[i].vx << " " << dynamic_obstacle_list_[i].vy << " "
                        << dynamic_obstacle_list_[i].vz << " ";
            }
            outfile << std::endl;
        }
    } else {
        if (tracker_info_flag_ and target_info_flag_) {
            std::ofstream outfile;
            outfile.open(state_history_file_name_, std::ios_base::app);
            outfile << time << " " << target_.px << " " << target_.py << " " << target_.pz << " " << target_.vx << " "
                    << target_.vy << " " << target_.vz << " ";
            for (int i = 0; i < tracker_list_.size(); i++) {
                outfile << tracker_list_[i].px << " " << tracker_list_[i].py << " " << tracker_list_[i].pz << " "
                        << tracker_list_[i].vx << " " << tracker_list_[i].vy << " " << tracker_list_[i].vz << " ";
            }
            outfile << std::endl;
        }
    }
}

double dmvc3d::Replayer::GetCurrentTime() {
    return ros::Time::now().toSec() - t0_;
}
