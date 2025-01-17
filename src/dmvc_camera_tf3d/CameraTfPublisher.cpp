//
// Created by larr-planning on 25. 1. 17.
//
#include <dmvc_camera_tf3d/CameraTfPublisher.h>

dmvc3d::CameraTfPublisher::CameraTfPublisher() : nh_("~") {
    target_state_subscriber_ = nh_.subscribe("/dmvc_simulator3d/target_state", 1, &CameraTfPublisher::TargetStateCallback,
                                             this);
    tracker_state_list_subscriber_ = nh_.subscribe("/dmvc_simulator3d/tracker_state_list", 1,
                                                   &CameraTfPublisher::TrackerStateListCallback, this);
    illuminate_publisher0_= nh_.advertise<sensor_msgs::Illuminance>("/illuminance3",1);
    illuminate_publisher1_= nh_.advertise<sensor_msgs::Illuminance>("/illuminance0",1);
    illuminate_publisher2_= nh_.advertise<sensor_msgs::Illuminance>("/illuminance1",1);
    illuminate_publisher3_= nh_.advertise<sensor_msgs::Illuminance>("/illuminance2",1);
}

void dmvc3d::CameraTfPublisher::Run() {
    ros::Rate loop_rate(50.0);
    while (ros::ok()) {
        PublishCameraTf();
        loop_rate.sleep();
        ros::spinOnce();
    }
}

void dmvc3d::CameraTfPublisher::PublishCameraTf() {
    static tf::TransformBroadcaster broadcaster;
    if (is_received_target_state_ and is_received_tracker_state_list_) {
        // Light TF (Under the trackers)
        {
//            for(int i =0;i<tracker_state_list_.size();i++){
//                tf::Transform transform;
//                transform.setOrigin(
//                        tf::Vector3(tracker_state_list_[i].x+0.5, tracker_state_list_[i].y+0.5, tracker_state_list_[i].z + 0.5));
//                tf::Quaternion q;
//                q.setX(0.0);
//                q.setY(0.0);
//                q.setZ(0.0);
//                q.setW(1.0);
//                q.normalize();
//                transform.setRotation(q);
//                broadcaster.sendTransform(
//                        tf::StampedTransform(transform, ros::Time::now(), "map", "light"+std::to_string(i)));
//            }
        }
        // Camera TF
        for (int i = 0; i < tracker_state_list_.size(); i++) {
            tf::Transform transform;
            transform.setOrigin(
                    tf::Vector3(tracker_state_list_[i].x, tracker_state_list_[i].y, tracker_state_list_[i].z));
            tf::Quaternion q;
            double rotation_matrix[3][3];
            rotation_matrix[0][0] = sin(tracker_state_list_[i].yaw);
            rotation_matrix[0][1] = 0;
            rotation_matrix[0][2] = cos(tracker_state_list_[i].yaw);
            rotation_matrix[1][0] = -cos(tracker_state_list_[i].yaw);
            rotation_matrix[1][1] = 0;
            rotation_matrix[1][2] = sin(tracker_state_list_[i].yaw);
            rotation_matrix[2][0] = 0;
            rotation_matrix[2][1] = -1;
            rotation_matrix[2][2] = 0;
            double trace = rotation_matrix[0][0] + rotation_matrix[1][1] + rotation_matrix[2][2];
            double s;

            if (trace > 0.0) {

                s = sqrt(1.0 + trace) * 2.0;
                q.setX((rotation_matrix[2][1] - rotation_matrix[1][2]) / s); //q.x
                q.setY((rotation_matrix[0][2] - rotation_matrix[2][0]) / s); //q.y
                q.setZ((rotation_matrix[1][0] - rotation_matrix[0][1]) / s); //q.z
                q.setW(0.25 * s);
                q.normalize();

            } else if ((rotation_matrix[0][0] > rotation_matrix[1][1]) and
                       (rotation_matrix[0][0] > rotation_matrix[2][2])) {

                s = (sqrt(1.0 + rotation_matrix[0][0] - rotation_matrix[1][1] - rotation_matrix[2][2])) * 2;
                q.setX(0.25 * s); //q.x
                q.setY((rotation_matrix[0][1] + rotation_matrix[1][0]) / s); //q.y
                q.setZ((rotation_matrix[0][2] + rotation_matrix[2][0]) / s); //q.z
                q.setW((rotation_matrix[2][1] - rotation_matrix[1][2]) / s);
                q.normalize();

            } else if (rotation_matrix[1][1] > rotation_matrix[2][2]) {

                s = (sqrt(1.0 + rotation_matrix[1][1] - rotation_matrix[0][0] - rotation_matrix[2][2])) * 2;
                q.setX((rotation_matrix[0][1] + rotation_matrix[1][0]) / s); //q.x
                q.setY(0.25 * s); //q.y
                q.setZ((rotation_matrix[1][2] + rotation_matrix[2][1]) / s); //q.z
                q.setW((rotation_matrix[0][2] - rotation_matrix[2][0]) / s);
                q.normalize();

            } else {

                s = (sqrt(1.0 + rotation_matrix[2][2] - rotation_matrix[0][0] - rotation_matrix[1][1])) * 2;
                q.setX((rotation_matrix[0][2] + rotation_matrix[2][0]) / s);
                q.setY((rotation_matrix[1][2] + rotation_matrix[2][1]) / s);
                q.setZ(0.25 * s);
                q.setW((rotation_matrix[1][0] - rotation_matrix[0][1]) / s);

            }
            transform.setRotation(q);

            broadcaster.sendTransform(
                    tf::StampedTransform(transform, ros::Time::now(), "map", "camera" + std::to_string(i)));
//            broadcaster.sendTransform(
//                        tf::StampedTransform(transform, ros::Time::now(), "map", "light"+std::to_string(i)));
//            sensor_msgs::Illuminance illum_msg;
//            illum_msg.header.stamp=ros::Time::now()-ros::Duration(0.05);
//            illum_msg.header.frame_id="light"+std::to_string(i);
//            illum_msg.illuminance = 500.0;
//            illum_msg.variance = 10.0;
//            if(i==0)
//                illuminate_publisher0_.publish(illum_msg);
//            else if(i==1)
//                illuminate_publisher1_.publish(illum_msg);
//            else if(i==2)
//                illuminate_publisher2_.publish(illum_msg);
//            else if(i==3)
//                illuminate_publisher3_.publish(illum_msg);
//            else
//            {
//
//            }
        }
    }

}

void dmvc3d::CameraTfPublisher::TargetStateCallback(const dmvc_tracker3d::ObjectState &msg) {
    target_state_.x = msg.px;
    target_state_.y = msg.py;
    target_state_.z = msg.pz;
    target_state_.yaw = 0.0;
    is_received_target_state_ = true;
}

void dmvc3d::CameraTfPublisher::TrackerStateListCallback(const dmvc_tracker3d::ObjectStateList &msg) {
    tracker_state_list_.clear();
    for (int i = 0; i < msg.object_state_list.size(); i++) {
        ObjectState temp_state;
        temp_state.x = msg.object_state_list[i].px;
        temp_state.y = msg.object_state_list[i].py;
        temp_state.z = msg.object_state_list[i].pz;
        if (is_received_target_state_)
            temp_state.yaw = atan2(target_state_.y - temp_state.y, target_state_.x - temp_state.x);
        else
            temp_state.yaw = 0.0;
        tracker_state_list_.push_back(temp_state);
    }
    is_received_tracker_state_list_ = true;
}