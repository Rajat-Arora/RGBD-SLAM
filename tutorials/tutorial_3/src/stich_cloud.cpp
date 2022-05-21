#include<iostream>
using namespace std;

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

// Eigen !
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct camera_params{

    double scale;
    double cx;
    double cy;
    double fx;
    double fy;
};

struct FRAME{ 
 	cv::Mat rgb, depth;  //The color map and depth map corresponding to this frame  
    cv::Mat desp;        //Feature descriptor  
    vector<cv::KeyPoint> kp;  
};  
 
struct RESULT_OF_PNP{ 
  	cv::Mat rvec, tvec; 
    int inliers; 
};

void computeKeyPointsAndDesp(FRAME& frame){

    cv::Ptr<cv::Feature2D> orb = cv::ORB::create();
    orb->detect( frame.rgb, frame.kp );
    orb->compute( frame.rgb, frame.kp, frame.desp );
    return;
} 

cv::Point3f point2dTo3d(const cv::Point3f& point, camera_params& camera){ //Here point.z contains the depth got from the RGBD Camera

    cv::Point3f p;
    p.z = double(point.z) / camera.scale;
    p.x = (point.x - camera.cx) * p.z / camera.fx;
    p.y = (point.y - camera.cy) * p.z / camera.fy;

    return p;
}



RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, camera_params& camera){

    vector<cv::DMatch> matches;
    cv::BFMatcher matcher;
    matcher.match(frame1.desp, frame2.desp, matches);

    cout<<"Find total "<<matches.size()<<" matches."<<endl;

    //Filter out matches based upon distance < 10*minimum distance
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }
    cout<<"min dis = "<<minDis<<endl;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < 10*minDis)
            goodMatches.push_back( matches[i] );
    }

    cout<<"good matches="<<goodMatches.size()<<endl;

    vector<cv::Point3f> pts_obj; //Vector of 3d points of frame1 in (x,y,z) not (u,v,d)
    vector<cv::Point2f> pts_img; //Vector of 2d points of frame2 (u,v)

     //Push filetered 2d points (u,v) of frame2 and (u,v,d) of frame1 
     //later on converted to (x,y,z) usinng func point2dTo3d
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );

        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera);
        pts_obj.push_back( pd );
    }

    // Create Camera matrix.
    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;

    //Computes rvec and tvec based upon reducing the reprojection error.
    //Computing R|t by reprojecting 3d points in prev frame to 2d points in curr frame.
    cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.999999999, inliers );

    cout<<"inliers: "<<inliers.rows<<endl;
    cout<<"R="<<rvec<<endl;
    cout<<"t="<<tvec<<endl;

    RESULT_OF_PNP result;
    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;
}

PointCloud::Ptr Image2PointCloud(const cv::Mat&rgb, const cv::Mat& depth, camera_params& camera){

    PointCloud::Ptr cloud ( new PointCloud );
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            ushort d = depth.ptr<ushort>(m)[n];
            if (d == 0)
                continue;
            PointT p;

            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;
    
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            cloud->points.push_back( p );
        }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

  return cloud;
}






int main(int argc, char** argv){


	FRAME frame1, frame2;	

	//Read the rgb images and the depth data.
	frame1.rgb = cv::imread( "../data/rgb1.png");
    frame2.rgb = cv::imread( "../data/rgb2.png");
    frame1.depth = cv::imread( "../data/depth1.png", -1);
    frame2.depth = cv::imread( "../data/depth2.png", -1);

	std::cout<<"Extracting features and descriptors"<<endl;
	computeKeyPointsAndDesp(frame1);
    computeKeyPointsAndDesp(frame2);

	camera_params C;
    C.cx = 325.5; C.cy = 253.5; C.fx = 518.0; C.fy = 519.0; C.scale = 1000.0;
    
    cout<<"solving pnp"<<endl;

	RESULT_OF_PNP result = estimateMotion( frame1, frame2, C);

	cout<<result.rvec<<endl<<result.tvec<<endl;

	cv::Mat R;
    cv::Rodrigues( result.rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);

	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

	Eigen::AngleAxisd angle(r);
    cout<<"translation"<<endl;
    Eigen::Translation<double,3> trans(result.tvec.at<double>(0,0), result.tvec.at<double>(0,1), result.tvec.at<double>(0,2));
    T = angle;
    T(0,3) = result.tvec.at<double>(0,0); 
    T(1,3) = result.tvec.at<double>(0,1); 
    T(2,3) = result.tvec.at<double>(0,2);

	cout<<"converting image to clouds"<<endl;
    PointCloud::Ptr cloud1 = Image2PointCloud( frame1.rgb, frame1.depth, C);
    PointCloud::Ptr cloud2 = Image2PointCloud( frame2.rgb, frame2.depth, C);

	cout<<"combining clouds"<<endl;
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *cloud1, *output, T.matrix() );
    *output += *cloud2;
    pcl::io::savePCDFile("../data/result.pcd", *output);
    cout<<"Final result saved."<<endl;

  //  pcl::visualization::CloudViewer viewer( "viewer" );
  //  viewer.showCloud( output );
  //  while( !viewer.wasStopped() )
  //  {
   //     
  //  }
    return 0;
}
