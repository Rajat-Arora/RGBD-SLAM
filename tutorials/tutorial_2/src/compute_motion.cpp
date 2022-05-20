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

struct camera_params{

    double scale;
    double cx;
    double cy;
    double fx;
    double fy;
};


cv::Point3f point2dTo3d(const cv::Point3f& point, camera_params& camera){ //Here point.z contains the depth got from the RGBD Camera

    cv::Point3f p;
    p.z = double(point.z) / camera.scale;
    p.x = (point.x - camera.cx) * p.z / camera.fx;
    p.y = (point.y - camera.cy) * p.z / camera.fy;

    return p;
}



int main(int argc, char** argv){

	//Read the rgb images and the depth data.
	cv::Mat rgb1 = cv::imread( "../data/rgb1.png");
    cv::Mat rgb2 = cv::imread( "../data/rgb2.png");
    cv::Mat depth1 = cv::imread( "../data/depth1.png", -1);
    cv::Mat depth2 = cv::imread( "../data/depth2.png", -1);

	
	//Create OpenCV detector and descriptor ptr
    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(); 
    //cout<< rgb2.size();
	//cv::imshow("Display window", rgb1); 
    // Detect kepypoints in frame 1 and frame2 and draw them

	vector<cv::KeyPoint> kp1, kp2;
	orb->detect( rgb1, kp1 );
    orb->detect( rgb2, kp2 );
    cout<<"Key points of two images: "<<kp1.size()<<", "<<kp2.size()<<endl;
    
	cv::Mat imgShow;
        
	cv::drawKeypoints(rgb1, kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("keypoints", imgShow);
    
    cv::imwrite("./data/keypoints.png", imgShow);
    cv::waitKey(0);
    
    //Compute descriptors and do matching between two frames using Brute Force matching.
	cv::Mat desp1, desp2;
    orb->compute(rgb1, kp1, desp1);
    orb->compute(rgb2, kp2, desp2);

    vector<cv::DMatch> matches; 
    cv::BFMatcher matcher;
    matcher.match(desp1, desp2, matches);
    cout<<"Find total "<<matches.size()<<" matches."<<endl;

	// Draw the unfiltered matches. There would be a lot of wrong matches
 	cv::Mat imgMatches;
    cv::drawMatches( rgb1, kp1, rgb2, kp2, matches, imgMatches );
    cv::imshow( "matches", imgMatches );
    cv::imwrite( "./data/matches.png", imgMatches );
    cv::waitKey( 0 );
    
    // Filter out matches based upon distance < 10*minimum distance
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

	//Draw good matches and print their size
	cout<<"good matches="<<goodMatches.size()<<endl;
    cv::drawMatches( rgb1, kp1, rgb2, kp2, goodMatches, imgMatches );
    cv::imshow( "good matches", imgMatches );
    cv::imwrite( "./data/good_matches.png", imgMatches );
    cv::waitKey(0);

	vector<cv::Point3f> pts_obj; //Vector of 3d points of frame1 in (x,y,z) not (u,v,d)
    vector<cv::Point2f> pts_img; //Vector of 2d points of frame2 (u,v)

	//Camera parameters struct to convert points from 2d to 3d. 
    camera_params C;
    C.cx = 325.5;
    C.cy = 253.5;
    C.fx = 518.0;
    C.fy = 519.0;
    C.scale = 1000.0;

    //Push filetered 2d points (u,v) of frame2 and (u,v,d) of frame1 later on converted to (x,y,z) usinng func point2dTo3d
	for (size_t i=0; i<goodMatches.size(); i++)
    {
        cv::Point2f p = kp1[goodMatches[i].queryIdx].pt;
        ushort d = depth1.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( kp2[goodMatches[i].trainIdx].pt ) );

        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, C );
        pts_obj.push_back( pd );
    }

    // Create Camera matrix.
	double camera_matrix_data[3][3] = {
        {C.fx, 0, C.cx},
        {0, C.fy, C.cy},
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

    //Show the inliers computed by RANSAC in solvePnPRansac function of OpenCv used for minimizing the reprojection error.
	vector< cv::DMatch > matchesShow;
    for (size_t i=0; i<inliers.rows; i++)
    {
        matchesShow.push_back( goodMatches[inliers.ptr<int>(i)[0]] );    
    }
    cv::drawMatches( rgb1, kp1, rgb2, kp2, matchesShow, imgMatches );
    cv::imshow( "inlier matches", imgMatches );
    cv::imwrite( "./data/inliers.png", imgMatches );
    cv::waitKey( 0 );
/*
*/	
return 0;
}
