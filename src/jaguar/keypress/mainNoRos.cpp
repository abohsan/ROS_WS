#include <X11/Xlib.h>
#include <iostream>
#include <memory>
#include "X11/keysym.h"



#include "../include/Jaguar.hpp"


/**
 *
 * @param ks  like XK_Shift_L, see /usr/include/X11/keysymdef.h
 * @return
 */
bool connactionState = false;

std::shared_ptr<Jaguar> jaguar = std::make_shared<Jaguar>("192.168.0.60", 10001);
// Jaguar *jaguar = new Jaguar("192.168.0.60", 10001);
void connectAndReleaseWheels(){
  if(!connactionState){
             std::cout << "Connecting ..." << std::endl;
             std::cout << "" << std::endl;
                if (jaguar->connect())
                {
                    std::cout << "Connected" << std::endl;

                    std::cout << "Releasing wheels ..." << std::endl;
                    jaguar->releaseWheels();
                    std::cout << "wheels released" << std::endl;
                    
                    std::cout << "Releasing flippers ..." << std::endl;       
                    jaguar->releaseRearFlipers();
                    jaguar->releaseFrontFlipers();
                    std::cout << "flippers released" << std::endl;

                    connactionState = true;
                }
                else
                {
                    std::cout << "Fail to connect" << std::endl;
                }
            }
}

bool key_is_pressed(KeySym ks)
{
    Display *dpy = XOpenDisplay(":0");
    char keys_return[32];
    XQueryKeymap(dpy, keys_return);
    KeyCode kc2 = XKeysymToKeycode(dpy, ks);
    bool isPressed = !!(keys_return[kc2 >> 3] & (1 << (kc2 & 7)));
    XCloseDisplay(dpy);
    return isPressed;
}

int main(int argc, char **argv)
{


    std::shared_ptr<Jaguar> jaguar = std::make_shared<Jaguar>("192.168.0.60", 10001);
    std::cout << "Please wait for the initialization" << std::endl;
    std::cout << "Initialization is done" << std::endl;
    while (true)
    {
        if (key_is_pressed(XK_q))
        {
            break;
        }
        else if (key_is_pressed(XK_z))
        {
          connectAndReleaseWheels();
        }
        else if (key_is_pressed(XK_2))
        {
            jaguar->lightOff();
            std::cout << "pressed 2" << std::endl;
        }
        else if (key_is_pressed(XK_3))
        {
            jaguar->lightON();
            std::cout << "pressed 3" << std::endl;
        }
        else if (key_is_pressed(XK_f))
        {
            // jaguar->forward();
        }
        else if (key_is_pressed(XK_b))
        {
            // jaguar->backward();
        }
        else if (key_is_pressed(XK_r))
        {
            // jaguar->turnRight();
        }
        else if (key_is_pressed(XK_l))
        {
            // jaguar->turnLeft();
        }
        else
        {
            // jaguar->stopWheels();
        }
    }

     jaguar->~Jaguar();
    return (0);
};