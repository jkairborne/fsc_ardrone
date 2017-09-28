#ifndef IBVS_H
#define IBVS_H

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include "ros/time.h"
#include "navdata_cb_ardrone.h"
#include "geometry_msgs/Twist.h"

typedef Eigen::Matrix<float, 8, 6> LeMat;
typedef Eigen::Matrix<float, 6, 8> LePlus;
typedef Eigen::Matrix<float, 8, 1> uv;
typedef Eigen::Matrix<float, 6, 1> velocity;

class IBVS {
    uv ImagePts,VImagePts, VCamPts, desiredPts, deltaS;
    LeMat Le;
    LePlus Le_psinv, DiagMat;
    //focal length is measured in pixels. bsln is in meters, and represents the line distance separating the two points in our z_est calculation
    double old_z_hat, z_est, focal_lngth, bsln;
    double Pinv_tolerance;
    double angleDes;
    int camWdth, camHght; // Just the height and width of the camera image
    cv::Point2f imageCenter; // store the center point of the image plane.
    velocity vc;
    bool correctDesiredPts;
    ros::Time tstart, tnow;
    navdata_cb_ardrone navdata;



    //function declarations
    double distance_calc(cv::Point2f pt1, cv::Point2f pt2);
    int within_bounds(int index, int totalind = -1);
    std::vector<cv::Point2f> uvToPoint2f(uv input);
    uv point2fToUv(std::vector<cv::Point2f> input);

    //void update_z_est();
  public:
    // Constructor:
    IBVS(double baseline = 0.15, double focal_length = 700.00, double camWidth = 640, double camHeight = 360);

    //Update functions
    void update_z_est(std::vector<cv::Point2f> pts);
    void update_z_est(double newEst);
    void update_uv_row (int update_pair, double new_u, double new_v, bool updateDesired = false);
    void update_uv (std::vector<cv::Point2f> uv_new, bool updateDesired = false);
    void update_uv (std::vector<double> uv_new, bool updateDesired = false); //Overload for vector of doubles
    void update_tolerance(double newval);
    void update_desiredPts(std::vector<cv::Point2f> new_desPts);
    void calc_desiredPts(double offsetx, double offsety, double psi = 0,cv::Point2f center = cv::Point2f(-1,-1));
    void update_Le_row(int, double);
    void update_Le(double);
    void manual_Le(std::vector<double> vecLe);
    void update_Le();
    void update_VImPts(Eigen::Matrix3d rotatM);
    void update_VCamPts();

    // calculate functions
    void MP_psinv_Le();
    geometry_msgs::Twist calculate_vc();
    void calculate_deltaS();
    std::vector<cv::Point2f> virtCam(std::vector<cv::Point2f> input, Eigen::Matrix3d rotatM);
    std::vector<cv::Point2f> imgToCam(std::vector<cv::Point2f> input);
    std::vector<cv::Point2f> camToImg(std::vector<cv::Point2f> input);
    uv camToImg(uv in);
    uv imgToCam(uv in);

    //display functions
    void display_Le();
    void display_LePlus();
    void display_params();
    void disp_uv_row(int n);
    void disp_uv();
    void addPtsToImg(cv::Mat& img, std::vector<cv::Point2f> ptsToAdd, cv::Scalar color = cv::Scalar(50.));

    std::vector<cv::Point2f> getDesPtsPt2F();
    std::vector<cv::Point2f> getVImPtsPt2F();
    uv getDesPtsEig();
    uv getVImPtsEig();
    double getZ_est();
    void rearrangeDesPts(std::vector<cv::Point2f> fourCorners);
};

#endif // IBVS_H
