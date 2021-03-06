#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// int i = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "myRobot_move_joint");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(100);

    sensor_msgs::JointState joint_state;

    joint_state.name.resize(11);
    joint_state.position.resize(11);

    joint_state.name[0] = "base_link__wheel_left";
    joint_state.name[1] = "base_link__wheel_right";
    joint_state.name[2] = "base_link__fliper_front_left";
    joint_state.name[3] = "base_link__fliper_back_left";
    joint_state.name[4] = "base_link__fliper_front_right";
    joint_state.name[5] = "base_link__fliper_back_right";
    joint_state.name[6] = "base_link__base_arm";
    joint_state.name[7] = "base_arm__arm_1";
    joint_state.name[8] = "arm_1__arm_2";
    joint_state.name[9] = "arm_2__arm_3";
    joint_state.name[10] = "arm_3__arm_4";

    while (ros::ok())
    {
        // if (i == 50)
        // {
        //     return 0;
        // }
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.position[0] = 0;
        joint_state.position[1] = 0;
        joint_state.position[2] = 0;
        joint_state.position[3] = 0;
        joint_state.position[4] = 0;
        joint_state.position[5] = 0;
        joint_state.position[6] = 0;
        joint_state.position[7] = 0;
        joint_state.position[8] = 0;
        joint_state.position[9] = 0;
        joint_state.position[10] = 0;
    
    
        //send the joint state and transform
        joint_pub.publish(joint_state);
        loop_rate.sleep();
        // i++;
    }
    return 0;
}