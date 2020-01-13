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
   if ( key_is_pressed(XK_0) && key_is_pressed(XK_Down))
    {
      if (sent != "m0d")
      {
        toBeSent << "m0d";
        sent = "m0d";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
    else if (key_is_pressed(XK_0) && key_is_pressed(XK_Up))
    {
      if (sent != "m0u")
      {
        toBeSent << "m0u";
        sent = "m0u";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    } 
    else if ( key_is_pressed(XK_1) && key_is_pressed(XK_Down))
    {
      if (sent != "m1d")
      {
        toBeSent << "m1d";
        sent = "m1d";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
    else if (key_is_pressed(XK_1) && key_is_pressed(XK_Up))
    {
      if (sent != "m1u")
      {
        toBeSent << "m1u";
        sent = "m1u";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    } 
    else if ( key_is_pressed(XK_2) && key_is_pressed(XK_Down))
    {
      if (sent != "m2d")
      {
        toBeSent << "m2d";
        sent = "m2d";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
    else if ( key_is_pressed(XK_2) && key_is_pressed(XK_Up))
    {
      if (sent != "m2u")
      {
        toBeSent << "m2u";
        sent = "m2u";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    } 
    else if ( key_is_pressed(XK_3) && key_is_pressed(XK_Down))
    {
      if (sent != "m3d")
      {
        toBeSent << "m3d";
        sent = "m3d";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
    else if ( key_is_pressed(XK_3) && key_is_pressed(XK_Up))
    {
      if (sent != "m3u")
      {
        toBeSent << "m3u";
        sent = "m3u";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    } 
    else if ( key_is_pressed(XK_4) && key_is_pressed(XK_Down))
    {
      if (sent != "m4d")
      {
        toBeSent << "m4d";
        sent = "m4d";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
    else if ( key_is_pressed(XK_4) && key_is_pressed(XK_Up))
    {
      if (sent != "m4u")
      {
        toBeSent << "m4u";
        sent = "m4u";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    } 
    else if ( key_is_pressed(XK_5) && key_is_pressed(XK_Down)  )
    {
      if (sent != "m5d")
      {
        toBeSent << "m5d";
        sent = "m5d";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
    else if (key_is_pressed(XK_5) && key_is_pressed(XK_Up)  )
    {
      if (sent != "m5u")
      {
        toBeSent << "m5u";
        sent = "m5u";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    } else if (key_is_pressed(XK_6) && key_is_pressed(XK_Up) )
    {
      if (sent != "m6u")
      {
        toBeSent << "m6u";
        sent = "m6u";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    } else if ( key_is_pressed(XK_7) && key_is_pressed(XK_Up) )
    {
      if (sent != "m7u")
      {
        toBeSent << "m7u";
        sent = "m7u";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }
     else if (key_is_pressed(XK_Left))
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
    else if (key_is_pressed(XK_o))
    {
      if (sent != "0")
      {
        toBeSent << "0";
        sent = "0";
        msg.data = toBeSent.str();
        chatter_pub.publish(msg);
      }
    }else if (key_is_pressed(XK_9))
    {
      if (sent != "9")
      {
        toBeSent << "9";
        sent = "9";
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
     else if (key_is_pressed(XK_i))
    {
      
        toBeSent << "i";
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
