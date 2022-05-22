#include"slam_base.h"


slam_base::slam_base(float scale, float fx, float fy, float cx, float cy):
	camera.scale(scale),
	camera.cx(cx),
	camera.cy(cy),
	camera.fx(fx),
	camera.fy(fy){

	return;
}


PointCloud::Ptr slam_base::Image2PointCloud(const cv::Mat&rgb, const cv::Mat& depth){

	PointCloud::Ptr cloud ( new PointCloud );
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            ushort d = depth.ptr<ushort>(m)[n];
            if (d == 0)
                continue;
            PointT p;

            p.z = double(d) / slam_base::camera.scale;
            p.x = (n - slam_base::camera.cx) * p.z / slam_base::camera.fx;
            p.y = (m - slam_base::camera.cy) * p.z / slam_base::camera.fy;
            
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


cv::Point3f slam_base::point2dTo3d(const cv::Point3f& point){ //Here point.z contains the depth got from the RGBD Camera

	cv::Point3f p;
    p.z = double(point.z) / slam_base::camera.scale;
    p.x = (point.x - slam_base::camera.cx) * p.z / slam_base::camera.fx;
    p.y = (point.y - slam_base::camera.cy) * p.z / slam_base::camera.fy;

    return p;
}

void slam_base::computeKeyPointsAndDesp(FRAME& frame){

	cv::Ptr<cv::Feature2D> orb = cv::ORB::create();
    orb->detect( frame.rgb, frame.kp );
	orb->compute( farame.rgb, frame.kp, frame.desp );
	return;
}  

RESULT_OF_PNP slam_base::estimateMotion(FRAME& frame1, FRAME& frame2){

	vector<cv::DMatch> matches;
    cv::BFMatcher matcher;
    matcher.match(frame1.desp, frame2.desp2, matches);

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
        cv::Point3f pd = point2dTo3d( pt );
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

Eigen::Isometry3d slam_base::cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec){

	cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ ) 
            r(i,j) = R.at<double>(i,j);
  
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(1,0); 
    T(2,3) = tvec.at<double>(2,0);
    return T;

}

PointCloud::Ptr slam_base::joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T){
	
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


