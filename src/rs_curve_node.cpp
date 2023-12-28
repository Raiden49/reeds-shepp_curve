/**
 * @file rs_curve_node.cpp
 * @author Yongmeng He
 * @brief 
 * @version 0.1
 * @date 2023-12-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <chrono>

#include "rs_curve/rs_curve_node.hpp"

namespace rs_curve
{
void RSCurveNode::InitPoseCallback(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    start_pos_.first.first = msg->pose.pose.position.x;
    start_pos_.first.second = msg->pose.pose.position.y;
    start_pos_.second = tf::getYaw(msg->pose.pose.orientation);

    init_pose_marker_.header.frame_id = "odom";
    init_pose_marker_.header.stamp = ros::Time::now();
    init_pose_marker_.ns = "init_pose_marker";
    init_pose_marker_.id = 0;
    init_pose_marker_.type = visualization_msgs::Marker::ARROW;
    init_pose_marker_.action = visualization_msgs::Marker::ADD;
    init_pose_marker_.pose.position = msg->pose.pose.position;
    init_pose_marker_.pose.orientation = msg->pose.pose.orientation;
    init_pose_marker_.scale.x = 1.0;
    init_pose_marker_.scale.y = 0.05;
    init_pose_marker_.scale.z = 0.05;
    init_pose_marker_.color.a = 1.0;
    init_pose_marker_.color.r = 0.0;
    init_pose_marker_.color.g = 1.0;
    init_pose_marker_.color.b = 0.0;

    ROS_INFO_STREAM("the init pose:" << start_pos_.first.first << ", "
            << start_pos_.first.second << ", " << start_pos_.second);
}

void RSCurveNode::GoalPoseCallback(
        const geometry_msgs::PoseStamped::ConstPtr& msg) {
    end_pos_.first.first = msg->pose.position.x;
    end_pos_.first.second = msg->pose.position.y;
    end_pos_.second = tf::getYaw(msg->pose.orientation);

    goal_pose_marker_.header.frame_id = "odom";
    goal_pose_marker_.header.stamp = ros::Time::now();
    goal_pose_marker_.ns = "goal_pose_marker";
    goal_pose_marker_.id = 0;
    goal_pose_marker_.type = visualization_msgs::Marker::ARROW;
    goal_pose_marker_.action = visualization_msgs::Marker::ADD;
    goal_pose_marker_.pose.position = msg->pose.position;
    goal_pose_marker_.pose.orientation = msg->pose.orientation;
    goal_pose_marker_.scale.x = 1.0;
    goal_pose_marker_.scale.y = 0.05;
    goal_pose_marker_.scale.z = 0.05;
    goal_pose_marker_.color.a = 1.0;
    goal_pose_marker_.color.r = 1.0;
    goal_pose_marker_.color.g = 0.0;
    goal_pose_marker_.color.b = 0.0;

    ROS_INFO_STREAM("the init pose:" << end_pos_.first.first << ", "
            << end_pos_.first.second << ", " << end_pos_.second);
}

visualization_msgs::Marker RSCurveNode::DrawRSCurve(
        const std::vector<point_type>& rs_pos_vec, const double& alpha= 0.15,
        const std::array<double, 3>& color = {1, 0, 0}) {

    visualization_msgs::Marker marker_path;
    marker_path.type = visualization_msgs::Marker::LINE_STRIP;
    marker_path.header.frame_id = "odom";
    marker_path.header.stamp = ros::Time::now();
    marker_path.ns = "odom";
    marker_path.id = 0;
    marker_path.action = visualization_msgs::Marker::ADD;
    marker_path.lifetime = ros::Duration();
    marker_path.color.r = color[0];
    marker_path.color.g = color[1];
    marker_path.color.b = color[2];
    marker_path.color.a = alpha;
    marker_path.scale.x = 0.02;
    marker_path.pose.orientation.w = 1.0;

    geometry_msgs::Point marker_point;
    for (auto rs_pos : rs_pos_vec) {
        marker_point.x = rs_pos.first.first;
        marker_point.y = rs_pos.first.second;
        marker_point.z = 0;
        marker_path.points.push_back(marker_point);
    }

    return marker_path;
}

void RSCurveNode::RSCurveProcess() {
    
    ROS_INFO("Start draw the rs curve");

    init_pose_sub_ = n_.subscribe(
            "/initialpose", 1, &RSCurveNode::InitPoseCallback, this);
    goal_pose_sub_ = n_.subscribe(
            "/move_base_simple/goal", 1, &RSCurveNode::GoalPoseCallback, this);

    // ros spins 10 frames per seconds
    ros::Rate loop_rate(10);

    auto rs_solver = std::make_shared<rs_curve::RSCurve>(1, start_pos_, end_pos_);
    auto rs_curve_vec_ex = rs_solver->GetRSCurve(start_pos_, end_pos_);
    std::vector<ros::Publisher> rs_marker_pub_vec;
    for (int i = 0; i < rs_curve_vec_ex.size(); i++) {
        rs_marker_pub_vec.push_back(n_.advertise<visualization_msgs::Marker>(
                rs_curve_vec_ex[i].first.second + "_marker", 1));
    }

    ros::Publisher best_rs_curve_pub = 
            n_.advertise<visualization_msgs::Marker>("best_curve_marker", 1);

    while (ros::ok()) {

        auto start_time = std::chrono::high_resolution_clock::now();

        init_pose_pub_.publish(init_pose_marker_);
        goal_pose_pub_.publish(goal_pose_marker_);

        rs_solver->start_pos_ = start_pos_;
        rs_solver->end_pos_ = end_pos_;

        auto rs_curve_vec = rs_solver->GetRSCurve(start_pos_, end_pos_);
        // ROS_INFO_STREAM("rs_curve_vec size:" << rs_curve_vec.size());
        auto best_curve = std::min_element(
                rs_curve_vec.begin(), rs_curve_vec.end(),
                [](const curve_type& curve1, const curve_type& curve2) {
                    return curve1.second.second < curve2.second.second;
                });
        size_t best_index = std::distance(rs_curve_vec.begin(), best_curve);
        // ROS_INFO_STREAM("best_index : " << best_index);
        for (int i = 0; i < rs_curve_vec.size(); i++) {
            auto rs_marker = i == best_index ? 
                    DrawRSCurve(rs_solver->GetRSPoint(
                                rs_curve_vec[i]), 1.0, {0, 1, 0}) : 
                    DrawRSCurve(rs_solver->GetRSPoint(rs_curve_vec[i]));
            if (i == best_index) {
                best_rs_curve_pub.publish(rs_marker);
            }
            else {
                rs_marker_pub_vec[i].publish(rs_marker);
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<
                std::chrono::microseconds>(end_time - start_time);
        ROS_INFO_STREAM("time spent:" << duration.count() / 1000 << "ms");

        rs_curve_vec.clear();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
}