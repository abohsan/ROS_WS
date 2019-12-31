#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int i = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "myRobot_move_joint");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(30);

    sensor_msgs::JointState joint_state;

    joint_state.name.resize(15);
    joint_state.position.resize(15);

    joint_state.name[0] = "base_link__link_01";
    joint_state.name[1] = "base_link__link_02";
    joint_state.name[2] = "base_link__link_03";
    joint_state.name[3] = "base_link__link_04";
    joint_state.name[4] = "base_link__link_05";
    joint_state.name[5] = "base_link__link_12";
    joint_state.name[6] = "base_link__link_13";
    joint_state.name[7] = "base_link__link_14";
    joint_state.name[8] = "base_link__link_15";
    joint_state.name[9] = "link_01__link_06";
    joint_state.name[10] = "link_06__link_07";
    joint_state.name[11] = "link_07__link_08";
    joint_state.name[12] = "link_08__link_09";
    joint_state.name[13] = "link_09__link_10";
    joint_state.name[14] = "link_09__link_11";

    while (ros::ok())
    {
        if (i == 200)
        {
            return 0;
        }
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
        joint_state.position[9] = 1.57;
        joint_state.position[10] = 0;
        joint_state.position[11] = 0;
        joint_state.position[12] = 0;
        joint_state.position[13] = 0;
        joint_state.position[14] = 0;

        //send the joint state and transform
        joint_pub.publish(joint_state);
        loop_rate.sleep();
        i++;
    }
    return 0;
}