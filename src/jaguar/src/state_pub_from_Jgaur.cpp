#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <jaguar/FlipMotor.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class joint_status
{
public:
  joint_status();

  private:
  void flipers_sub_Callback(const jaguar::FlipMotor::ConstPtr &msg);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;

  ros::Subscriber joy_sub_;
  ros::Publisher joint_pub;
};

joint_status::joint_status()
{

  joy_sub_ = nh_.subscribe<jaguar::FlipMotor>("flipers", 10, &joint_status::flipers_sub_Callback, this);
  joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

}

void joint_status::flipers_sub_Callback(const jaguar::FlipMotor::ConstPtr &msg)
{
  sensor_msgs::JointState joint_state;

    joint_state.name.resize(15);
    joint_state.position.resize(15);
    joint_state.name[0] = "base_link__base_arm";
    joint_state.name[1] = "base_link__wheel_left";
    joint_state.name[2] = "base_link__wheel_right";
    joint_state.name[3] = "base_link__fliper_front_left";
    joint_state.name[4] = "base_link__fliper_back_left";
    joint_state.name[5] = "base_link__fliper_front_right";
    joint_state.name[6] = "base_link__fliper_back_right";
    joint_state.name[7] = "base_arm__arm_1";
    joint_state.name[8] = "arm_1__arm_2";
    joint_state.name[9] = "arm_2__arm_3";
    joint_state.name[10] = "arm_3__arm_4";
    joint_state.name[11] = "arm_4__griper_1";
    joint_state.name[12] = "arm_4__griper_2";
        
        
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = 0;
    joint_state.position[1] = 0;
    joint_state.position[2] = 0;
    joint_state.position[3] = 0;
    joint_state.position[4] = 0;
    joint_state.position[5] = msg->go_to_leftFront;
    joint_state.position[6] = msg->go_to_leftRear;
    joint_state.position[7] = msg->go_to_rightFront;
    joint_state.position[8] = msg->go_to_rightRear;
    joint_state.position[9] = 0;
    joint_state.position[10] = 0;
    joint_state.position[11] = 0;
    joint_state.position[12] = 0;

    joint_pub.publish(joint_state);
}
int i = 0;




int main(int argc, char **argv)
{
    // ros::init(argc, argv, "myRobot_move_joint");
    // ros::NodeHandle n;
    // ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    // // ros::Subscriber flipers_sub = n.subscribe("/flipers", 1, flipers_sub_Callback);

    // ros::Rate loop_rate(30);

    // sensor_msgs::JointState joint_state;

    // joint_state.name.resize(15);
    // joint_state.position.resize(15);

    // joint_state.name[0] = "base_link__link_01";
    // joint_state.name[1] = "base_link__link_02";
    // joint_state.name[2] = "base_link__link_03";
    // joint_state.name[3] = "base_link__link_04";
    // joint_state.name[4] = "base_link__link_05";
    // joint_state.name[5] = "base_link__link_12";
    // joint_state.name[6] = "base_link__link_13";
    // joint_state.name[7] = "base_link__link_14";
    // joint_state.name[8] = "base_link__link_15";
    // joint_state.name[9] = "link_01__link_06";
    // joint_state.name[10] = "link_06__link_07";
    // joint_state.name[11] = "link_07__link_08";
    // joint_state.name[12] = "link_08__link_09";
    // joint_state.name[13] = "link_09__link_10";
    // joint_state.name[14] = "link_09__link_11";

    // while (ros::ok())
    // {
    //     if (i == 200)
    //     {
    //         return 0;
    //     }
    //     //update joint_state
    //     joint_state.header.stamp = ros::Time::now();
    //     joint_state.position[0] = 0;
    //     joint_state.position[1] = 0;
    //     joint_state.position[2] = 0;
    //     joint_state.position[3] = 0;
    //     joint_state.position[4] = 0;
    //     joint_state.position[5] = 0;
    //     joint_state.position[6] = 0;
    //     joint_state.position[7] = 0;
    //     joint_state.position[8] = 0;
    //     joint_state.position[9] = 1.57;
    //     joint_state.position[10] = 0;
    //     joint_state.position[11] = 0;
    //     joint_state.position[12] = 0;
    //     joint_state.position[13] = 0;
    //     joint_state.position[14] = 0;

    //     //send the joint state and transform
    //     joint_pub.publish(joint_state);
    //     loop_rate.sleep();
    //     i++;
    // }
    return 0;
}