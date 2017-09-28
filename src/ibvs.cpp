#define _USE_MATH_DEFINES

#include "ibvs.h"
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include "geometry_msgs/Twist.h"


geometry_msgs::Twist IBVS::calculate_vc()
{
    geometry_msgs::Twist vcRet;
    vc = Le_psinv * deltaS;

    // Swap x and y for coordinate frame issues:
    double temp = vc(1,0);
    vc(1,0) = vc(0,0);
    vc(0,0) = temp;

   // display_params();
    vcRet.linear.x = vc(0,0);
    vcRet.linear.y = vc(1,0);
    vcRet.linear.z = vc(2,0);
    vcRet.angular.x = 0.1;
    vcRet.angular.y = 0;
    vcRet.angular.z = vc(5,0);
    return vcRet;
}

void IBVS::calculate_deltaS()
{
    deltaS = VImagePts - desiredPts;
//std::cout << "Just before navdata get rpy\n";//32

    navdata.get_rpy();
//std::cout << "Just after navdata get rpy\n";//32
}

void IBVS::display_params()
{
    std::cout << "Desired image : ";
    for (int i=0;i<desiredPts.rows();i++) { std::cout << desiredPts(i,0) << "  ";}
    std::cout << '\n';
    std::cout << "Image Points : ";
    for (int i=0;i<ImagePts.rows();i++) { std::cout << ImagePts(i,0)<< "  ";}
    std::cout << '\n';
    std::cout << "DeltaS : ";
    for (int i=0;i<deltaS.rows();i++) { std::cout << deltaS(i,0)<< "  ";}
    std::cout << '\n';
    std::cout << "z_est : " << z_est << '\n';

    std::cout << "v_desired : ";
    for (int i=0;i<vc.rows();i++) { std::cout << vc(i,0)<< "  ";}
    std::cout << '\n';
}


std::vector<cv::Point2f> IBVS::getDesPtsPt2F()
{

    std::vector<cv::Point2f> output(desiredPts.rows()/2);
    for(int i = 0 ; i < desiredPts.rows()/2 ; i++)
    {
        output[i] = cv::Point2f(desiredPts(2*i),desiredPts(2*i+1));
    }
    return output;
}

std::vector<cv::Point2f> IBVS::getVImPtsPt2F()
{
    std::vector<cv::Point2f> output(VCamPts.rows()/2);
    for(int i = 0 ; i < VCamPts.rows()/2 ; i++)
    {
        output[i] = cv::Point2f(VImagePts(2*i),VImagePts(2*i+1));
    }
    return output;
}

uv IBVS::getDesPtsEig()
{
    return desiredPts;
}

uv IBVS::getVImPtsEig()
{
    return VImagePts;
}

double IBVS::getZ_est()
{
    return z_est;
}

double IBVS::distance_calc(cv::Point2f pt1, cv::Point2f pt2)
{
    double result;

    result = sqrt((pt2.x-pt1.x)*(pt2.x-pt1.x)+(pt2.y-pt1.y)*(pt2.y-pt1.y));
    return result;
} // End distance_calc


void IBVS::rearrangeDesPts(std::vector<cv::Point2f> fourCorners)
{
    int numpts = fourCorners.size();
    std::vector<cv::Point2f> desPt2f(4);

    desPt2f = uvToPoint2f(desiredPts);
    desPt2f.resize(4);

    double angleCurrent, angleDiff;

    angleCurrent = atan2((fourCorners[1].y - fourCorners[0].y),(fourCorners[1].x-fourCorners[0].x));

//    for(int i=0;i<8;i++) {std::cout << desiredPts[i] << " ";}

//Watch out, this goes increasing angles downwards.
    angleDiff = angleCurrent - angleDes;
    int offsetx,offsety;
    offsetx = 40;
    offsety = 28;

    if(angleDiff >= 3*M_PI/4 || angleDiff < -3*M_PI/4)
    {
        calc_desiredPts(offsetx,offsety,M_PI);
    }
    else if(angleDiff >= M_PI/4)
    {
        calc_desiredPts(offsetx,offsety,M_PI/2);
    }
    else if(angleDiff <= -M_PI/4)
    {
        calc_desiredPts(offsetx,offsety,-M_PI/2);
    }
    else
    {
        calc_desiredPts(offsetx,offsety);
    }
    // We only want this function to be called once to rearrange.
    // It is set to true if the chessboard is lost from the image, back to false here.
} // end rearrangeDesPts

//This function is simply meant to help with the wraparound of the whole
int IBVS::within_bounds(int index, int totalind)
{
    int numpts;
    numpts = totalind;
    if(totalind= -1) {int numpts = desiredPts.rows();}
    if (index >= 2*numpts)
    {
        std::cout << "Index is twice what it should be";
    }
// If the index is greater than numpoints, that means that we want to be wrapping around the matrix
    else if (index >= numpts)
    {
        return (index-numpts);
    }
    else {return index;}
}

std::vector<cv::Point2f> IBVS::uvToPoint2f(uv input)
{
    int ptnum = input.rows()/2;
    std::vector<cv::Point2f> output(ptnum);

//    std::cout << "inside uvToPoint2f: \n" << "ptnum: " << ptnum<< " input: " << input << '\n';

    for(int i = 0; i<ptnum;i++)
    {
        output[i].x = input(2*i,0);
        output[i].y = input(2*i+1,0);
//        std::cout<< "in for loop: i = " << i << " output(x)/output(y): " << output[i].x << "/" << output[i].y;
    }

    return output;
}

uv IBVS::point2fToUv(std::vector<cv::Point2f> input)
{
    uv output;

    for(int i = 0; i<input.size();i++)
    {
        output(2*i,0) = input[i].x;
        output(2*i+1,0) = input[i].y;
    }

    return output;
}

std::vector<cv::Point2f> IBVS::virtCam(std::vector<cv::Point2f> input, Eigen::Matrix3d rotatM)
{
    std::vector<cv::Point2f> outputs;

    double z_virt;
    // Here need to hope that z_est has been recently updated;

    for(int i = 0 ; i< input.size(); i++ )
    {
        cv::Point2f fromCenter = input[i] - imageCenter;
        Eigen::Vector3d realCoords, virtCoords;
        realCoords(2,0) = z_est;
        realCoords(1,0) = z_est * fromCenter.y/focal_lngth;
        realCoords(0,0) = z_est * fromCenter.x/focal_lngth;

        virtCoords = rotatM * realCoords;

        cv::Point2f temp;
        z_virt = virtCoords(2,0);

        temp.y = virtCoords(1,0)*focal_lngth/virtCoords(2,0);
        temp.x = virtCoords(0,0)*focal_lngth/virtCoords(2,0);

//        outputs.push_back(cv::Point2f(temp.x+imageCenter.y,temp.y+imageCenter.x));
        outputs.push_back(cv::Point2f(temp+imageCenter));
    }

    return outputs;
}


std::vector<cv::Point2f> IBVS::camToImg(std::vector<cv::Point2f> input)
{
    std::vector<cv::Point2f> outputs;
    for(int i = 0 ; i< input.size(); i++ )
    {
        outputs.push_back(cv::Point2f(input[i]+imageCenter));
    }
    return outputs;
}

uv IBVS::camToImg(uv in)
{
    int row_number1, row_number2;
    uv out;
    for(int i=0;i<4;i++)
    {
       row_number1 = 2*(i-1);
       row_number2 = 2*i -1;
       out(row_number1,0) = in(0,0)+camWdth/2;
       out(row_number2,0) = in(1,0)+camHght/2;
    }
    return out;
}

uv IBVS::imgToCam(uv in)
{
    int row_number1, row_number2;
    uv out;
   //       std::cout <<  "in imgToCam: ";
    for(int i=0;i<4;i++)
    {
       row_number1 = 2*(i);
       row_number2 = 2*i +1;
       out(row_number1,0) = in(row_number1,0)-camWdth/2;
       out(row_number2,0) = in(row_number2,0)-camHght/2;
//      std::cout <<  "out(" << row_number1 <<"): " << out(row_number1,0);
 //     std::cout <<  "out(" << row_number2 <<"): " << out(row_number2,0);
    }
//    std::cout << "in: " << in << " out: "<<  out << "\n";
    return out;
}



std::vector<cv::Point2f> IBVS::imgToCam(std::vector<cv::Point2f> input)
{
    std::vector<cv::Point2f> outputs;
    for(int i = 0 ; i< input.size(); i++ )
    {
        outputs.push_back(cv::Point2f(input[i]-imageCenter));
    }
    return outputs;
}



void IBVS::update_z_est(std::vector<cv::Point2f> pts)
{
    double delta_x1, delta_y1, delta_x2, delta_y2, dist1, dist2, avg_dist;

    dist1 = cv::norm(pts[3]-pts[0]);
    dist2 = cv::norm(pts[2]-pts[1]);
    
    avg_dist = 0.5*(dist1+dist2);

    z_est = bsln*focal_lngth/avg_dist;
    //    std::cout << '\n' << z_est << "\n";
}

void IBVS::update_z_est(double newEst)
{
    z_est = newEst;
}



void IBVS::MP_psinv_Le()
{
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(Le, Eigen::ComputeFullU | Eigen::ComputeFullV);

    int i = 0;
    for(i=0;i<6;i++)
    {
        // Here we have an If clause, simply on the chance that the values are too close to zero
        // This is a main reason for things being slow in calculating the jacobian, the singular values being near zero but not quite zero
        // leads to the values in the matrix blowing up.

        if(svd.singularValues()[i] > Pinv_tolerance || svd.singularValues()[i]<(-Pinv_tolerance))
        {
            DiagMat(i,i) = (1/svd.singularValues()[i]);
        }//endif
        else{ DiagMat(i,i) = 0;}//end else
    }//end for

    //chose not to use the diagonal matrix because it has issues with being displayed, and can't handle a rectangular matrix
    //   Eigen::DiagonalMatrix<float, 6> SingValMat(6);
    //   SingValMat.diagonal() = svd.singularValues();

    Eigen::MatrixXf V, UT;
    V = svd.matrixV();
    UT = svd.matrixU().transpose();

    Le_psinv = V*DiagMat*UT;
}

void IBVS::display_Le()
{
	std::cout << Le;
}

void IBVS::display_LePlus()
{
    std::cout << Le_psinv;
}


// n should be a number between 1 and 4, depending on the "blob" we want to keep track of.
void IBVS::update_Le_row(int n, double z_hat)
{
   int row_number1, row_number2;
   row_number1 = 2*(n-1);
   row_number2 = 2*n -1;
   Le(row_number1,0) = -1/z_hat;
   Le(row_number1,2) = VCamPts(row_number1,0) / z_hat;
   Le(row_number1,3) = VCamPts(row_number1,0) * VCamPts(row_number2,0);
   Le(row_number1,4) = - (1 + VCamPts(row_number1,0)*VCamPts(row_number1,0));
   Le(row_number1,5) = VCamPts(row_number2,0);
//Populate second row of that particular pair 
   Le(row_number2,1) = -1/z_hat;
   Le(row_number2,2) = VCamPts(row_number2,0) / z_hat;
   Le(row_number2,3) = 1 + VCamPts(row_number1,0)*VCamPts(row_number1,0);
   Le(row_number2,4) = -VCamPts(row_number1,0) * VCamPts(row_number2,0);
   Le(row_number2,5) = -VCamPts(row_number1,0);
}

void IBVS::update_Le(double z_hat)
{
	IBVS::update_Le_row(1,z_hat);
	IBVS::update_Le_row(2,z_hat);
	IBVS::update_Le_row(3,z_hat);
    IBVS::update_Le_row(4,z_hat);
}

void IBVS::manual_Le(std::vector<double> vecLe)
{
    LeMat newLe;

    for(int i=0;i<8;i++)
    {
        for(int j=0; j<6; j++)
        {
            newLe(i,j) = vecLe[i+j];
        }
    }
    Le = newLe;
}

void IBVS::update_Le()
{
    update_Le(z_est);
}

void IBVS::update_VImPts(Eigen::Matrix3d rotatM)
{
    VImagePts = point2fToUv(virtCam(uvToPoint2f(ImagePts),rotatM));
}

void IBVS::update_VCamPts()
{
    VCamPts = imgToCam(VImagePts);
}

void IBVS::update_uv (std::vector<cv::Point2f> uv_new, bool updateDesired)
{
    if(uv_new.size() != 4) {std::cout << "Your update_uv function call does not contain 4 pairs";}
    else
    {
       if (updateDesired)
       {
           for(int j=0;j<uv_new.size();j++)
           {
               desiredPts(j*2,0) = uv_new[j].x;
               desiredPts(j*2+1,0) = uv_new[j].y;
           }
       }
       else
       {
            for(int j=0;j<uv_new.size();j++)
            {
                ImagePts(j*2,0) = uv_new[j].x;
                ImagePts(j*2+1,0) = uv_new[j].y;
            }
       }
    }
}

void IBVS::update_uv (std::vector<double> uv_new, bool updateDesired)
{
    if(uv_new.size() != 8) {std::cout << "Your update_uv double function call does not contain 8 numbers";}
    else
    {
        if(updateDesired)
        {
            for(int j=0;j<uv_new.size();j++)
            {
                desiredPts(j,0) = uv_new[j];
            }

        }
        else
        {
            for(int j=0;j<uv_new.size();j++)
            {
                ImagePts(j,0) = uv_new[j];
            }

        }
    }
}

void IBVS::update_uv_row (int update_pair, double new_u, double new_v, bool updateDesired)
{
    if(updateDesired)
    {
        desiredPts(update_pair*2,0) = new_u;
        desiredPts(update_pair*2+1,0) = new_v;
    }
    else
    {
        ImagePts(update_pair*2,0) = new_u;
        ImagePts(update_pair*2+1,0) = new_v;
    }
}


IBVS::IBVS(double baseline, double focal_length, double camWidth, double camHeight)
{
    Pinv_tolerance = 0.000001;
    correctDesiredPts = true;
    focal_lngth = focal_length;
    bsln = baseline;
    angleDes = 0; // sets the angle between points 0 and 1 desired
    camWdth = camWidth;
    camHght = camHeight;
    imageCenter = cv::Point2f(camWdth/2,camHght/2);
    desiredPts << 270,125,370,125,270,195,370,195;
    tstart = ros::Time::now();

  //  navdata = navdata_cb_ardrone();

    ImagePts << 1,2,3,4,5,6,7,8;

    Le <<	0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0;
	DiagMat <<	0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0;
	Le_psinv <<	0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0;
}

void IBVS::update_tolerance(double newval)
{
    Pinv_tolerance = newval;
}

void IBVS::update_desiredPts(std::vector<cv::Point2f> new_desPts)
{
    if(new_desPts.size() != 4) {std::cout << "Your update_desiredPts function call does not contain 4 pairs";}
    else
    {
       for(int j=0;j<new_desPts.size();j++)
       {
           ImagePts(j*2,0) = new_desPts[j].x;
           ImagePts(j*2+1,0) = new_desPts[j].y;
       }
    }
}

void IBVS::calc_desiredPts(double offsetx, double offsety, double psi, cv::Point2f center)
{
    Eigen::Matrix<double,2,4> vecMat, rVecMat1;

    Eigen::Matrix<double,2,2> rotMat;
    if(center == cv::Point2f(-1,-1)) {center = imageCenter;}

    rotMat(0,0) = cos(psi);
    rotMat(1,1) = cos(psi);
    rotMat(1,0) = sin(psi);
    rotMat(0,1) = -sin(psi);


    vecMat << -offsetx,offsetx,-offsetx,offsetx,\
            -offsety,-offsety,offsety,offsety;

    rVecMat1 = rotMat*vecMat;

    std::vector<cv::Point2f> desPtNew(4);
    desPtNew[0] = center+ cv::Point2f(rVecMat1(0,0),rVecMat1(1,0));
    desPtNew[1] = center+ cv::Point2f(rVecMat1(0,1),rVecMat1(1,1));
    desPtNew[2] = center+ cv::Point2f(rVecMat1(0,2),rVecMat1(1,2));
    desPtNew[3] = center+ cv::Point2f(rVecMat1(0,3),rVecMat1(1,3));
   // desiredPts =
    desiredPts << desPtNew[0].x, desPtNew[0].y , desPtNew[1].x, desPtNew[1].y,desPtNew[2].x, desPtNew[2].y,desPtNew[3].x, desPtNew[3].y;
}

void IBVS::disp_uv_row(int n)
{
std::cout << "\n Pair " << (n+1) << ": " <<"u = " << ImagePts(2*n,0) << ", v = " << ImagePts(2*n+1,0)<< '\n';;
}

void IBVS::disp_uv()
{
    for(int i=0;i<ImagePts.size();i++)
    {
        std::cout << ImagePts[i] << " ";
    }//end for
    std::cout << std::endl;
}

void IBVS::addPtsToImg(cv::Mat& img, std::vector<cv::Point2f> ptsToAdd, cv::Scalar color)
{
    for(int i = 0; i<ptsToAdd.size(); i++)
    {
        if(ptsToAdd[i].x <0 || ptsToAdd[i].x > camWdth || ptsToAdd[i].y < 0 || ptsToAdd[i].y > camHght)
        {
           // std::cout << "A point to be added to the image was out of range";
            std::string display_string;
            std::stringstream out;
            out << "point out of image";
            display_string = out.str();

            //Add numbering to the four points discovered.
            cv::putText( img, display_string, cv::Point2f(10,10), CV_FONT_HERSHEY_COMPLEX, 1,color, 1, 1);
        }
        else
        {
            cv::circle(img, ptsToAdd[i], 1, color, -1 );
            std::string display_string;
            std::stringstream out;
            out << i;
            display_string = out.str();

            //Add numbering to the four points discovered.
            cv::putText( img, display_string, ptsToAdd[i], CV_FONT_HERSHEY_COMPLEX, 1,color, 1, 1);
        } // end else
    }// end for
}// end addPtsToImg
