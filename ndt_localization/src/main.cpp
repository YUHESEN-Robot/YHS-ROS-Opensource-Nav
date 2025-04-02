#include "ndt_localization_node.hpp"

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "ndt_localization_node");

    // 创建NdtLocalization对象
    NdtLocalization ndt_localization;

    // 进入ROS循环
    ros::spin();

    return 0;
}
