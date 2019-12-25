#include "ros/ros.h"
#include "std_msgs/String.h"

#include <X11/Xlib.h>
#include <iostream>
#include "X11/keysym.h"

using namespace std;
string sent;
bool key_is_pressed(KeySym ks)
{
  Display *dpy = XOpenDisplay(":0");
  char keys_return[31];
  XQueryKeymap(dpy, keys_return);
  KeyCode kc2 = XKeysymToKeycode(dpy, ks);
  bool isPressed = !!(keys_return[kc2 >> 3] & (1 << (kc2 & 7)));
  XCloseDisplay(dpy);
  return isPressed;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "keyboard_command");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("keyboard_command", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream toBeSent;
    if (key_is_pressed(XK_Left))
    {
      if (sent != "l")
      {
        toBeSent << "l";
        sent = "l";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
    else if (key_is_pressed(XK_Right))
    {
      if (sent != "r")
      {
        toBeSent << "r";
        sent = "r";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
    else if (key_is_pressed(XK_Down))
    {
      if (sent != "b")
      {
        toBeSent << "b";
        sent = "b";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
    else if (key_is_pressed(XK_0))
    {
      if (sent != "0")
      {
        toBeSent << "0";
        sent = "0";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }else if (key_is_pressed(XK_1))
    {
      if (sent != "1")
      {
        toBeSent << "1";
        sent = "1";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }else if (key_is_pressed(XK_o))
    {
      if (sent != "o")
      {
        toBeSent << "o";
        sent = "o";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
    else if (key_is_pressed(XK_Up))
    {
      if (sent != "f")
      {
        toBeSent << "f";
        sent = "f";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
    else if (key_is_pressed(XK_x))
    {
      
        toBeSent << "x";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
    
    }
     else if (key_is_pressed(XK_z))
    {
      
        toBeSent << "z";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
    
    }
    else
    {
      if (sent != "s")
      {
        toBeSent << "s";
        sent = "s";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
