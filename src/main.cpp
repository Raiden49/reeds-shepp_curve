/**
 * @file main.cpp
 * @author Yongmeng He
 * @brief 
 * @version 0.1
 * @date 2023-12-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "rs_curve/rs_curve_node.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "rs_curve");
    ros::NodeHandle n;
    auto node = new rs_curve::RSCurveNode(n);
    node->RSCurveProcess();

    return 0;
}