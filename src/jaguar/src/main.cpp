
#include "../include/Nodes.hpp"
#include <string>

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "Jagaur");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    jaguar_ns::Nodes node(nh, nh_private);
    ROS_INFO("Initialized a multi-thread node.");
    ros::MultiThreadedSpinner s(4);   // Use 4 threads
    ROS_INFO_STREAM("Main loop in thread:" << boost::this_thread::get_id());   
    ros::spin(s);
}

