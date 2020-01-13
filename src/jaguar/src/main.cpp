
#include "../include/Nodes.hpp"
#include <string>

// void mune()
// {
//     print("     choose from the following:");
//     print("         0. run both jaguar and its arm");
//     print("         1. to run jaguar ");
//     print("         2. run the arm");
// }
int main(int argc, char **argv)
{

    // mune();
    // char chosen = getchar();
    ros::init(argc, argv, "Jagaur");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    jaguar_ns::Nodes node(nh, nh_private);
    ros::MultiThreadedSpinner s(4);   // Use 4 threads
    ROS_INFO_STREAM("Main loop in thread:" << boost::this_thread::get_id());
    ros::spin(s);
    // return 0;
}
