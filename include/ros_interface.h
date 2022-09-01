#ifndef __ROS_INTERFACE__
#define __ROS_INTERFACE__

#include <iostream>
#include "state_variable.h"
#include "eskf.h"

using namespace std;

class ROS_Interface
{
    private:
    public:
    ROS_Interface();
    ~ROS_Interface();
};

/***********************************************************************
 * Initialize 
 **********************************************************************/
ROS_Interface::ROS_Interface()
{
    cout << "ROS_Interface Start!" << endl;
}

ROS_Interface::~ROS_Interface()
{
    cout << "ROS_Interface Finish" << endl;
}

#endif // ROS_INTERFACE