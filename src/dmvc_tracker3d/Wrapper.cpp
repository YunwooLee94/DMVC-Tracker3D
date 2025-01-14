//
// Created by larr-planning on 25. 1. 13.
//
#include <dmvc_tracker3d/Wrapper.h>

void dmvc3d::Wrapper::Run() {
    thread_planner_ = thread(&Wrapper::RunPlanning, this);
    ROS_INFO_ONCE("THREAD FOR PLANNING CREATED.");
    thread_ros_wrapper_ = thread(&RosWrapper::RunRos, ros_wrapper_ptr_);
    ROS_INFO_ONCE("THREAD FOR RUNROS CREATED.");
    thread_planner_.join();
    thread_ros_wrapper_.join();
}

dmvc3d::Wrapper::Wrapper():p_base_shared_(std::make_shared<dmvc3d::PlannerBase>()) {
    ros_wrapper_ptr_ = new RosWrapper(p_base_shared_);
    ros_wrapper_ptr_->InitSubscriberAndPublisher();
    tracker_ = new Tracker(ros_wrapper_ptr_->GetTrackingParam(),p_base_shared_);
//    visualizer_ = new PlanningVisualizer(ros_wrapper_ptr_->GetVisualizationParam());
}

void dmvc3d::Wrapper::RunPlanning() {
    ros::Rate loop_rate(ros_wrapper_ptr_->GetPlanningFrequency());
    while(ros::ok()){
        bool planning_success = tracker_->Plan(ros_wrapper_ptr_->GetCurrentTime());
//        cout<<"FINISH PLANNING"<<endl;
        tracker_->UpdateResultToBase(planning_success);
//        cout<<"UPDATE PLANNING"<<endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
}