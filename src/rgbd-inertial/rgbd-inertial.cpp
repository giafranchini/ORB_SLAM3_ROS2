#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-inertial-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam rgbdi path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor mte;
    
    bool visualization = true;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, visualization);

    auto node = std::make_shared<RgbdInertialSlamNode>(&SLAM);
    std::cout << "============================ " << std::endl;

    mte.add_node(node);
    mte.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
