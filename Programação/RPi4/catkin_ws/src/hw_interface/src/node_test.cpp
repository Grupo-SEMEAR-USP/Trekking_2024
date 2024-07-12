#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "hw_interface");
    ros::NodeHandle nh; // Cria um NodeHandle

    ROS_INFO("Oi Jamaica");
    std::cout<<"Oi brasil";

    return 0;
}