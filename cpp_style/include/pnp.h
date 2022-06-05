#pragma once

#include "common_headers.h"
#include "rgbdframe.h"
#include "parameter_reader.h"
#include "orb.h"

namespace rgbd_slam{

struct PNP_INFORMATION{

	int numFeatureMatches = 0;
    int numInliers = 0;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); 
};

class PnPSolver{

public:
    PnPSolver( const rgbd_slam::ParameterReader& para, const rgbd_slam::OrbFeature& orbFeature ):
        parameterReader( para ),
        orb( orbFeature )
    {
        min_inliers = para.getData<int>("pnp_min_inliers");
        min_match = para.getData<int>("pnp_min_matches");
    }

	// Solve the pnp problem 
    // Input 2d point, 3d point, camera internal parameters 
    // Output transformation matrix T (initial value can be set), inliers index 
    // return success 
	bool solvePnP( const vector<cv::Point2f>& img, const vector<cv::Point3f>& obj, const rgbd_slam::CAMERA_INTRINSIC_PARAMETERS& camera, vector<int>& inliersIndex,
                      Eigen::Isometry3d& transform );

	 // Frame solution: given two frames directly, matching is calculated internally 
    // input two frames 
    // output transformation matrix and inlier index	
	bool solvePnPFrame( const rgbd_slam::RGBDFrame::Ptr& frame1, const rgbd_slam::RGBDFrame::Ptr& frame2, PNP_INFORMATION& pnp_information, bool drawMatches=false );

protected:
    const rgbd_slam::ParameterReader& parameterReader;
    const rgbd_slam::OrbFeature& orb;

    int min_inliers =10;
    int min_match   =30;

};
}
