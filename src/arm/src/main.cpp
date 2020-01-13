#include "../include/Nodes.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Arm");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    arm_ns::Nodes node(nh, nh_private);
    ros::MultiThreadedSpinner s(4);   // Use 4 threads
    ROS_INFO_STREAM("Main loop in thread:" << boost::this_thread::get_id());
    ros::spin(s);
}
