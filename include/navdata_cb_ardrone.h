#ifndef NAVDATA_CB_ARDRONE_H
#define NAVDATA_CB_ARDRONE_H


#include <ros/ros.h>
#include <vector>
#include <iostream>
#include "std_msgs/String.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "Eigen/Dense"

class navdata_cb_ardrone
{
    ros::NodeHandle nh_;
    ros::Subscriber navdata_sub_;
    int count;
	
    double roll, pitch, yaw;
    // Function declarations
    void callback(const ardrone_autonomy::Navdata& msg);    
public:
    navdata_cb_ardrone();
    std::vector<double> get_rpy();
    Eigen::Matrix3d getRotM();

    void set_rpy(double newroll, double newpitch, double newyaw);
}; // End class


#endif // NAVDATA_CB_ARDRONE_H
