#include<iostream>
#include <string>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


class slam_base{

 public:
 slam_base(float scale, float fx, float fy, float cx, float cy);
 
 // Used to convert the complete image to point cloud
 PointCloud::Ptr Image2PointCloud(const cv::Mat& rgb, const cv::Mat &depth);

 cv::Point3f point2dTo3d(const cv::Point3f& point);
 
 struct FRAME{ 
	cv::Mat rgb, depth;  //The color map and depth map corresponding to this frame  
	cv::Mat desp;        //Feature descriptor  
	vector<cv::KeyPoint> kp;  
 }; 
 
 struct RESULT_OF_PNP{ 
	 cv::Mat rvec, tvec; 
     int inliers; 
 };

 //  computeKeyPointsAndDesp extracts key points and feature descriptors at the same time  
 void computeKeyPointsAndDesp(FRAME& frame);

 //  estimateMotion calculates the motion between two frames  
 //  Input: frame 1 and frame 2, camera  
 RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2);
 
 // cvMat2Eigen
 Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec);
 
 // joinPointCloud 
 PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T);
	

 private:

//  Frame structure 
 struct camera_params{
	
    const double scale; 
    const double cx; 
    const double cy; 
    const double fx; 
    const double fy;
};
		
 camera_params camera; //Object of camera parameters struct

 typedef pcl::PointXYZRGBA PointT;   //Defining the point cloud to be stored.
 typedef pcl::PointCloud<PointT> PointCloud; 

};
