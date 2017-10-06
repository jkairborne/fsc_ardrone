#include "pid.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "OptiTools.h"
#include "ardrone_autonomy/Navdata.h"
#include <fsc_ardrone/pbvs.h>


#include <dynamic_reconfigure/server.h>
#include <fsc_ardrone/PIDsetConfig.h>


#include <stdio.h>
#include <math.h>

//#define USEOPTI
//#define USEVICON

#define USEROOMBA_VEL
#define USEROOMBA_ACC


#define ZDES 1.0 //desired height in m

#define KPLAT 0.035
#define KDLAT 0.25

#define TARGETSCALE (8/(100*22.2))
#define FX 670
#define FY 670
#define X0 500//470
#define Y0 445//387.4


double saturate_bounds(double max, double min, double val);
void printnavdata(ardrone_autonomy::Navdata msg);
void virtcam(double origImgPts[], double camRoll, double camPitch);

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher pbvs_topic; //Send a Twist message to the
ros::Publisher srcCmd;
ros::Publisher new_gains;
ros::Subscriber ardrone_subscriber;
#if defined(USEROOMBA_VEL) || defined(USEROOMBA_ACC)
ros::Subscriber roomba_subscriber;
#endif
ros::Time newtime, oldtime;
ros::Duration dt_ros;
bool targetVisible;
    bool visibleInLast;
std_msgs::Float64 cmdNotPBVS, cmdPBVS;
geometry_msgs::TwistStamped newgains;

double oldvel_r;
ros::Time newtime_r, oldtime_r;
ros::Duration dt_ros_r;

    // Define controller gains
    double Kp_x = KPLAT;
    double Kd_x = KDLAT;
    double Ki_x = 0.0;
    double Kp_y = KPLAT;
    double Kd_y = KDLAT;
    double Ki_y = 0.0;
    double Kp_z = 0.01;
    double Kd_z = 0;
    double Ki_z = 0;
    double dt = 0.03333; //tag detection takes place onboard at 30Hz
    double Kp_psi = 0.01;
    double Kd_psi = 0.0;
    double Ki_psi = 0.0;

    // Define controller setpoints, in case there is no subscriber to callback
    double x_des = 0.0;
    double y_des = 0.0;
    double z_des = 0.6;
    // Create the PID class instances for x, y, and z:
    PID pidx = PID(dt,1,-1,Kp_x,Kd_x,Ki_x);
    PID pidy = PID(dt,1,-1,Kp_y,Kd_y,Ki_y);
    PID pidz = PID(dt,1,-1,Kp_z,Kd_z,Ki_z);
    PID pidpsi = PID(dt,1,-1,Kp_psi,Kd_psi,Ki_psi);

#ifdef USEROOMBA_VEL
    double Kp_x_vel = 0.1;
    double Kd_x_vel = 0.0;
    double Ki_x_vel = 0.0;
    PID pidx_vel = PID(dt,1,-1,Kp_x_vel,Kd_x_vel,Ki_x_vel);
    double x_vel_component = 0;
#endif
#ifdef USEROOMBA_ACC
    double Kp_x_acc = 0.1;
    double Kd_x_acc = 0.0;
    double Ki_x_acc = 0.0;
    PID pidx_acc = PID(dt,1,-1,Kp_x_acc,Kd_x_acc,Ki_x_acc);
    double x_acc_component;
#endif

    // This is the callback from the parameter server
void callback(ardronecontrol::PIDsetConfig &config, uint32_t level) {
//  ROS_INFO("Reconfigure Request: %f %f", 
//             config.Kp_x,confi    /*
    // Save the new configuration to doubles
      Kp_y = config.Kp_y;
      Kd_y = config.Kd_y;
      Ki_y = config.Ki_y;
      // Call the mod_params function of the Pimpl class - this then calls the set_gains function in the PID class which actually changes the gains used for calculations
      pidy.mod_params(Kp_y, Kd_y,Ki_y);
      // Change the desired positions
      y_des = config.set_y;


// Save the new configuration to doubles
  Kp_x = config.Kp_x;
  Kd_x = config.Kd_x;
  Ki_x = config.Ki_x;
  // Call the mod_params function of the Pimpl class - this then calls the set_gains function in the PID class which actually changes the gains used for calculations
  pidx.mod_params(Kp_x,Kd_x,Ki_x);
  pidy.mod_params(Kp_x,Kd_x,Ki_x);
  // Change the desired positions
  x_des = config.set_x;

#ifdef USEROOMBA_VEL
  Kp_x_vel = config.Kp_x_vel;
  Kd_x_vel = config.Kd_x_vel;
  Ki_x_vel = config.Ki_x_vel;
  pidx_vel.mod_params(Kp_x_vel,Kd_x_vel,Ki_x_vel);
#endif
#ifdef USEROOMBA_ACC
  Kp_x_acc = config.Kp_x_acc;
  Kd_x_acc = config.Kd_x_acc;
  Ki_x_acc = config.Ki_x_acc;
  pidx_acc.mod_params(Kp_x_acc,Kd_x_acc,Ki_x_acc);
#endif

//std::cout << "new gains: kp,d,i x: " << Kp_x << " " << Kd_x  << " " << Ki_x  << " " << "kp,d,i y: " << Kp_y << " " << Kd_y << " " << Ki_y << '\n';
  // Create and publish the new gains to a TwistStamped method:
  newgains.header.stamp = ros::Time::now();
  newgains.twist.linear.x = Kp_x;
  newgains.twist.linear.y = Kd_x;
  newgains.twist.linear.z = Ki_x;
  newgains.twist.angular.x = Kp_x;
  newgains.twist.angular.y = Kd_x;
  newgains.twist.angular.z = Ki_x;
  new_gains.publish(newgains);
}


// Main function. rectifies coordinate system, converts quaternion to rpy, 
// converts from world to body frame, applies PIDs to the channels, then
// outputs the message onto a "/cmd_vel" topic.

void MsgCallback(const ardrone_autonomy::Navdata msg)
{
  //  printnavdata(msg);
    newtime = msg.header.stamp;
    dt_ros = newtime-oldtime;
    dt = dt_ros.toSec();
    //std::cout << "received NalastTransitionvdata message\n";

    // Create the output mess650.87age to be published
    fsc_ardrone::pbvs pbvsmsg;


    if(msg.tags_count ==0)
    {
        pbvsmsg.targetInSight = false;
        pbvsmsg.delX = 0;
        pbvsmsg.delY = 0;
        pbvsmsg.delZ = 0;
        if(targetVisible) // this means target was seen in last frame but not in this one
        {
            srcCmd.publish(cmdNotPBVS);
        }
        //std::cout << "target lost: x: " << pid_output.linear.x << " y: " << pid_output.linear.y << '\n';
        pbvs_topic.publish(pbvsmsg); // this is probably redundant
        targetVisible = 0;

        return;
    }
    else
    {
        pbvsmsg.targetInSight=true;
    }


    if(targetVisible == 0) // This means target was not visible in last frame, but is in this one.
    {
        pidx.rst_integral();
        pidy.rst_integral();
        pidz.rst_integral();
        pidpsi.rst_integral();

        srcCmd.publish(cmdPBVS);
        dt = 0; //This will use the default time step specified when the PID was created - and prevents too large of one being used.
        targetVisible = 1;
    }

    double xtag, ytag, xpos0, ypos0,zpos0, delta_x,delta_y,delta_z,delta_psi;

    xtag = (double) msg.tags_xc[0]; // necessary to avoid int/double errors
    ytag = (double) msg.tags_yc[0];

    xpos0 = (xtag-X0)/FX;
    ypos0 = (ytag-Y0)/FY;
    zpos0 = msg.tags_distance[0]*TARGETSCALE;// See sept 9th 2017 notes

    //Psi requires somewhat special treatment, because for ArDrone it gets reported in degrees, from 0 to 360
    //delta_psi = (180-msg.tags_orientation[0]); // original, "proper" orientation
    delta_psi = msg.tags_orientation[0]-90;
    if ((delta_psi)>180)
    {
        delta_psi = 180-delta_psi;
    }

    double orig[2], virt[2];
    orig[0] = xpos0;
    orig[1] = ypos0;
    orig[2] = zpos0;

    virtcam(orig,(-msg.rotY*M_PI/180), (-msg.rotX*M_PI/180));

    xpos0 = orig[0];
    ypos0 = orig[1];
    zpos0 = orig[2];

 //   delta_x = ((xpos0 * zpos0)-0.047)/1.5997;
 //   delta_y = ((ypos0 * zpos0)+0.0439)/2.1789;

    delta_x = orig[0];
    delta_y = orig[1];
    //std::cout << "deltax, deltay: " << delta_x << " " << delta_y << "\n";

    // Populate the output message
    //Note that we swap deltax and deltay because of coordinate system differences
    pbvsmsg.delX = delta_y;
    pbvsmsg.delY = delta_x;
    pbvsmsg.delZ = orig[2];

    pbvsmsg.outX = pidx.calculate(0,delta_y,dt);
    pbvsmsg.outY = pidy.calculate(0,delta_x,dt);
    pbvsmsg.outZ = pidx.calculate(0,delta_z,dt);

    pbvsmsg.outPsi = -pidpsi.calculate(0,delta_psi,dt);

    oldtime = newtime;

#ifdef USEROOMBA_ACC

    pbvsmsg.fwdTgtddot = x_acc_component; // this assumes the rover is moving along y axis
#endif
#ifdef USEROOMBA_VEL
    pbvsmsg.fwdTgtdot = x_vel_component;// this assumes the rover is moving along y axis
#endif

    pbvs_topic.publish(pbvsmsg);
}

void roombaCallback(const geometry_msgs::TwistStamped& velcmd)
{
    ros::Time newtime_r = velcmd.header.stamp;

    dt_ros_r = newtime_r - oldtime_r;
    double timediff_r = dt_ros_r.toSec();
    double newvel_r = velcmd.twist.linear.x;
#ifdef USEROOMBA_ACC
    double acc = (newvel_r-oldvel_r)/timediff_r;
    x_acc_component =pidx_acc.calculate(0,acc,timediff_r);
    oldvel_r = newvel_r;

//    std::cout << "line 240: x_acc_component = " << x_acc_component << " , dt_ros_r =" << timediff_r << '\n';
#endif
#ifdef USEROOMBA_VEL
    x_vel_component = pidx_vel.calculate(0,velcmd.twist.linear.x,timediff_r);
//        std::cout << "line 244: x_vel_component = " << x_vel_component << " , vel_cmd =" << velcmd.twist.linear.x<< '\n';
#endif
    oldtime_r = newtime_r;
}

void virtcam(double origImgPts[],double camRoll, double camPitch)
{
    double u0 = origImgPts[0];
    double v0 = origImgPts[1];
    double z_est = origImgPts[2];
    double x = ((u0*z_est)-0.047)/1.5997;
    double y = ((v0*z_est)+0.0439)/2.1789;
   //std::cout << "x_est: " << x << " y_est: " << y << '\n';

    double x_v = cos(camPitch)*x + sin(camRoll)*sin(camPitch)*y + cos(camRoll)*sin(camPitch)*z_est;
    double y_v = cos(camRoll)*y-sin(camRoll)*z_est;
    double z_v = -sin(camPitch)*x+cos(camPitch)*sin(camRoll)*y+cos(camPitch)*cos(camRoll)*z_est;

    origImgPts[0] = x_v;//z_v;
    origImgPts[1] = y_v;//z_v;
    origImgPts[2] = z_v;
    //std::cout << "after virtcam: x_est: " << x_v << " y_est: " << y_v << '\n';

    //std::cout << "u0,v0: " << u0 << " " << v0 << " x,y " << x << " " << y << " x_v, y_v: " << x_v << " " << y_v << " output: " << origImgPts[0] << " " << origImgPts[1] << '\n';

}

double saturate_bounds(double max, double min, double val)
{
//    std::cout << "in saturate bounds - max: " << max << " min: " << min << " val: " << val << '\n';
    if(max < min)
    {
        std::cout << "Your Min is greater than your max in saturate_bounds fct!";
        return -1;
    }
    if(val>max){return max;}
    else if (val<min) {return min;}
    else {return val;}
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "ArdronePID");
    ros::NodeHandle n;

std::cout << "In beginning";

    // Advertise the cmd vel node
    pbvs_topic = n.advertise<fsc_ardrone::pbvs>("pbvsData", 1);
    srcCmd = n.advertise<std_msgs::Float64>("src_cmd",1);

    new_gains = n.advertise<geometry_msgs::TwistStamped>("gain_changer", 1);
cmdPBVS.data = 1;
cmdNotPBVS.data = 0;

    // These four lines set up the dynamic reconfigure server
    dynamic_reconfigure::Server<ardronecontrol::PIDsetConfig> server;
    dynamic_reconfigure::Server<ardronecontrol::PIDsetConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);


    oldtime = ros::Time::now();
    // Subscribe to the Ardrone data incoming from the OptiTrack
    ardrone_subscriber = n.subscribe("/ardrone/navdata", 1, MsgCallback);
#if defined(USEROOMBA_VEL) || defined(USEROOMBA_ACC)
    roomba_subscriber = n.subscribe("/roomba_vel_cmd",1,roombaCallback);
#endif
    ros::spin();


    return 0;
}

void printnavdata(ardrone_autonomy::Navdata msg)
{
    if(msg.tags_count>0)
    {
        std::cout << "x/y/z/psi: " << msg.tags_xc[0] << "/" << msg.tags_yc[0] << "/" << msg.tags_distance[0] << "/" << msg.tags_orientation[0] << "\n";
    }
}


