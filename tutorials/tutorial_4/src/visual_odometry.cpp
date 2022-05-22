#include <iostream>
#include <fstream>
#include <sstream>
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

PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, camera_params& camera){ 
    
    PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;

    static pcl::VoxelGrid<PointT> voxel;
    float gridsize = 0.01f; //1 cm change according to requirement.
    voxel.setLeafSize(gridsize, gridsize, gridsize);
    voxel.setInputCloud( newCloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    return tmp;

}


// Read frame based on index
FRAME readFrame( int index );
// Take norm
double normofTransform( cv::Mat rvec, cv::Mat tvec );


int main(int argc, char** argv){

	int start_index = 0; //To be updated
	int end_index = 600; 

	cout<<"Initializing ..."<<endl;

	int currIndex = startIndex;

	FRAME lastFrame = readFrame(currIndex);

	camera_params C;
    C.cx = 325.5; C.cy = 253.5; C.fx = 518.0; C.fy = 519.0; C.scale = 1000.0;

	computeKeyPointsAndDesp( lastFrame );   
    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, C);
 
    cout<<"solving pnp"<<endl;

	RESULT_OF_PNP result = estimateMotion( frame1, frame2, C);

	//Check if need to make obj of visualizer

	//Optimize these values
	// These decide if the match that we make was successfull of it should be discarded.
	int min_inliers = 5; //Remove the frames with fewer inliers in solvePnPRASNAC
    double max_norm = 0.3; //Remove the case where the obtained transformation matrix is too large. Because the motion is assumed to be continuous, the two frames are not too far apart 
	//This value has been calculated by norm( delta t ) + min(2 *pi* norm(r), norm(r) ). Norm sum of displacement and rotation. If it is greater than max_norm we consider matching error.

	for ( currIndex = startIndex+1; currIndex < endIndex; currIndex++ )
    {
	
		cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame( currIndex );
		
		computeKeyPointsAndDesp(currFrame);
	
		RESULT_OF_PNP result = estimateMotion( lastFrame, currFrame, camera );
		
        //Checks to remove mismatched frames
		if ( result.inliers < min_inliers ) 
            continue;

        double norm = normofTransform(result.rvec, result.tvec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
            continue;

		Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
        cout<<"T="<<T.matrix()<<endl;
        
        cloud = joinPointCloud( cloud, currFrame, T, camera );
        
        if ( visualize == true )
            viewer.showCloud( cloud );

        lastFrame = currFrame;
    }

    pcl::io::savePCDFile( "../data/result.pcd", *cloud );
    return 0;
}



FRAME readFrame( int index)
{
    FRAME f;
    //Change name of directories
    string rgbDir   =  "ABC";
    string depthDir =  "XYZ";
    
    string rgbExt   =   "OPQ";
    string depthExt =   "RST";

    stringstream ss;
    ss<<rgbDir<<index<<rgbExt;
    string filename;
    ss>>filename;
    f.rgb = cv::imread( filename );

    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;

    f.depth = cv::imread( filename, -1 );
    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}
