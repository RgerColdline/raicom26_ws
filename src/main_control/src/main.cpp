#include <ros/ros.h>
#include "mission_manager.h"

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "mission_state_machine");
    ros::NodeHandle nh("~");

    MissionManager manager(nh);
    manager.initROSCommunication();
    manager.waitForConnection();

    std::cout << "按回车键开始任务..." << std::endl;
    std::cin.get();

    manager.run();
    return 0;
}