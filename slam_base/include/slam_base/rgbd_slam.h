//TODO: Studied lesson3 feature detection and solvepnp to implement and complete by today eod
//TODO: Studied lesson3 class to reader parameters from text file and store into map implement and complete by today eod
//TODO: Studied lesson3 different tutorials for each concept implement and complete by today eod
//TODO: Studied lesson4 trnasform and stich point cloud implement and complete by today eod

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
 
 PointCloud::Ptr Image2PointCloud(const cv::Mat& rgb, const cv::Mat &depth);

 cv::Point3f point2dTo3d(const cv::Point3f& point);
 
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
