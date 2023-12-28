/**
 * @file rs_curve_node.hpp
 * @author Yongmeng He
 * @brief 
 * @version 0.1
 * @date 2023-12-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef RS_CURVE_NODE_HPP_
#define RS_CURVE_NODE_HPP_

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include "rs_curve/rs_curve.hpp"

namespace rs_curve 
{
class RSCurveNode {
    public:
        /**
         * @brief Construct a new RSCurveNode object
         * 
         * @param n 
         */
        RSCurveNode(ros::NodeHandle& n) : n_(n){

            init_pose_pub_ = n_.advertise
                    <visualization_msgs::Marker>("init_pose_marker", 1);
            goal_pose_pub_ = n_.advertise
                    <visualization_msgs::Marker>("goal_pose_marker", 1);
        }
        ~RSCurveNode() = default;

        /**
         * @brief 在RVIZ中可视化RS曲线
         * 
         * @param rs_pos_vec RS某条曲线的所有点
         * @param alpha 不透明度
         * @param color 颜色
         * @return visualization_msgs::Marker 
         */
        visualization_msgs::Marker DrawRSCurve(
                const std::vector<point_type>& rs_pos_vec, const double& alpha,
                const std::array<double, 3>& color);
        /**
         * @brief 程序主函数
         * 
         */
        void RSCurveProcess();
        /**
         * @brief 获取初始位置点的回调函数，由rviz人为设置，并绘制在RVIZ上
         * 
         * @param msg 初始点的信息
         */
        void InitPoseCallback(
                const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        /**
         * @brief 获取目标位置点的回调函数，由rviz人为设置，并绘制在RVIZ上
         * 
         * @param msg 目标点的信息
         */
        void GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    public:
        point_type start_pos_, end_pos_;

    private:
        ros::NodeHandle n_;
        ros::Subscriber init_pose_sub_, goal_pose_sub_;
        ros::Publisher init_pose_pub_, goal_pose_pub_;
        visualization_msgs::Marker init_pose_marker_, goal_pose_marker_;
};
}

#endif // RS_CURVE_NODE_HPP_