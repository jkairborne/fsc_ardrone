/*
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
*/
#include <iostream>
#include <ctype.h>
#include "ibvs.h"
#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>
#include "navdata_cb_ardrone.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include "fsc_ardrone/ImgData.h"

static const int CBOARD_COL = 5;
static const int CBOARD_ROW = 4;

using namespace std;
using namespace cv;

class ImageConverter
{
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    geometry_msgs::Twist empTwist;

    ros::Publisher pub2_;
    std_msgs::Int16 cmdFromIBVS, cmdNotFromIBVS;

    ros::Publisher pub3_;
    fsc_ardrone::ImgData ImgData;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    IBVS ibvs;
    navdata_cb_ardrone navdataCb;
    bool correctDesiredPts;
    Mat storedImage;

    geometry_msgs::Twist cmdToSend;
    int scalingLat, scalingVert, scalingPsi;
    int count;
    // Function declarations
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void processImage(Mat &image);
    void fill_ImgData(uv virt, uv des, double z_hat);
public:
    ImageConverter(const std::string &navdataCbTopic = "/ardrone/navdata")
        : it_(nh_)
    {
        ibvs = IBVS();
        correctDesiredPts = true;
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/ardrone/image_raw", 1,
           &ImageConverter::imageCb, this);
        pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_IBVS", 1);
        pub2_ = nh_.advertise<std_msgs::Int16>("/src_cmd", 1);
        pub3_ = nh_.advertise<fsc_ardrone::ImgData>("/img_data", 1);
        cmdFromIBVS.data = 2;
        cmdNotFromIBVS.data = 1;
        count=0;

        scalingLat = 8000;
        scalingVert = 2;
        scalingPsi = 2;

        empTwist.linear.x =0;
        empTwist.linear.y =0;
        empTwist.linear.z =0;
        empTwist.angular.x =0;
        empTwist.angular.y =0;
        empTwist.angular.z =0;
    }
}; // End class

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}


void ImageConverter::processImage(Mat &image)
{
    vector<Point2f> corners;
    Size chessSize(CBOARD_COL,CBOARD_ROW);
    namedWindow("Image2");
    count+=1;
    bool patternfound = findChessboardCorners(image, chessSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    if(patternfound)
    {
        Mat gray;
        cvtColor(image,gray,CV_BGR2GRAY);
        cornerSubPix(gray,corners,Size(11,11),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+ CV_TERMCRIT_ITER,30,0.1));

        // 0 4 15 19...
        std::vector<Point2f> fourCorners;
        fourCorners.push_back(corners[0]);
        fourCorners.push_back(corners[CBOARD_ROW]);
        fourCorners.push_back(corners[CBOARD_ROW*(CBOARD_COL-1)-1]);
        fourCorners.push_back(corners[CBOARD_COL*CBOARD_ROW-1]);
        fourCorners.resize(4);

        if(correctDesiredPts)
        {
            ibvs.rearrangeDesPts(fourCorners);
            correctDesiredPts = false;
            pub2_.publish(cmdFromIBVS);
        }

        // ibvs.calc_desiredPts(50,35);
        ibvs.addPtsToImg(image,fourCorners);
        ibvs.addPtsToImg(image,ibvs.getDesPtsPt2F(),cv::Scalar(100,100,100));
        ibvs.update_uv(fourCorners);


        //Want to update z_est just before calling virtual camera fct because it needs height
        ibvs.update_z_est(fourCorners);
        ibvs.update_VImPts(navdataCb.getRotM());
        ibvs.update_VCamPts();
        ibvs.addPtsToImg(image,ibvs.getVImPtsPt2F(),cv::Scalar(150,150,0));
        ibvs.calculate_deltaS();

        ibvs.update_Le();
        ibvs.MP_psinv_Le();
        cmdToSend = ibvs.calculate_vc();


        // Manipulate and send the command:
        cmdToSend.linear.x = cmdToSend.linear.x /scalingLat;
        cmdToSend.linear.y = cmdToSend.linear.y /scalingLat;
        cmdToSend.linear.z = cmdToSend.linear.z /scalingVert;
        cmdToSend.angular.x = 0.1;
        cmdToSend.angular.y = 0;
        cmdToSend.angular.z = cmdToSend.angular.z/scalingPsi;

        pub_.publish(cmdToSend);

        fill_ImgData(ibvs.getVImPtsEig(),ibvs.getDesPtsEig(),ibvs.getZ_est());
        pub3_.publish(ImgData);
        imshow("Image2",image);

        cv::waitKey(3);
    }
    else
    {
    imshow("Image2",image);
    if(!correctDesiredPts){pub2_.publish(cmdNotFromIBVS);}
    pub_.publish(empTwist); // if image no longer visible, send empty command
    correctDesiredPts = true;
    cv::waitKey(3);
    }
}

void ImageConverter::fill_ImgData(uv virt, uv des, double z_hat)
{
    ImgData.x0 = virt(0,0);
    ImgData.y0 = virt(1,0);
    ImgData.x1 = virt(2,0);
    ImgData.y1 = virt(3,0);
    ImgData.x2 = virt(4,0);
    ImgData.y2 = virt(5,0);
    ImgData.x3 = virt(6,0);
    ImgData.y3 = virt(7,0);
    ImgData.desx0 = des(0,0);
    ImgData.desy0 = des(1,0);
    ImgData.desx1 = des(2,0);
    ImgData.desy1 = des(3,0);
    ImgData.desx2 = des(4,0);
    ImgData.desy2 = des(5,0);
    ImgData.desx3 = des(6,0);
    ImgData.desy3 = des(7,0);
    ImgData.z_est = z_hat;
    ImgData.x_c = (virt(0,0)+virt(2,0) + virt(4,0)+virt(6,0))/4;
    ImgData.y_c = (virt(1,0)+virt(3,0) + virt(5,0)+virt(7,0))/4;
    ImgData.desx_c = (des(0,0)+des(2,0) + des(4,0)+des(6,0))/4;
    ImgData.desy_c = (des(1,0)+des(3,0) + des(5,0)+des(7,0))/4;

/*    printf("\n x,desx, y,desy0: %f %f %f %f \n",ImgData.x0,virt(0,0),ImgData.y0,virt(1,0));
    printf("x,desx, y,desy1: %f %f %f %f \n",ImgData.x1,virt(2,0),ImgData.y1,virt(3,0));
    printf("x,desx, y,desy2: %f %f %f %f \n",ImgData.x2,virt(4,0),ImgData.y2,virt(5,0));
    printf("x,desx, y,desy3: %f %f %f %f \n",ImgData.x3,virt(6,0),ImgData.y3,virt(7,0));
    printf("\n%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f, centers: %f %f ",ImgData.x0,ImgData.y0,ImgData.x1,ImgData.y1,ImgData.x2,\
           ImgData.y2,ImgData.x3,ImgData.y3,ImgData.desx0,ImgData.desy0,ImgData.desx1,ImgData.desy1,ImgData.desx2,\
           ImgData.desy2,ImgData.desx3,ImgData.desy3,ImgData.x_c,ImgData.y_c);*/

}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    storedImage = cv_ptr->image;
    processImage(storedImage);
}

