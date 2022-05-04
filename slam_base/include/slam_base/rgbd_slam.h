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
//TODO: Initialize camera_params in constructor. Correct camera param variables in slam_base.cpp
 slam_base();
 
 PointCloud::Ptr point2d_to_3d(const cv::Mat& rgb, const cv::Mat &depth);
 
 struct camera_params{
	
    const double camera_factor; 
    const double camera_cx; 
    const double camera_cy; 
    const double camera_fx; 
    const double camera_fy;
};
		


 }
 


 typedef pcl::PointXYZRGBA PointT;   //Defining the point cloud to be stored.
 typedef pcl::PointCloud<PointT> PointCloud; 

};
