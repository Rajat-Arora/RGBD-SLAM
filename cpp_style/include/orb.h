# pragma once
# include "common_headers.h" 
# include "rgbdframe.h" 
# include "../Thirdparty/orbslam_modified/include/ORBextractor.h"
# include "../Thirdparty/orbslam_modified/include/Converter.h"


namespace rgbd_slam{

class OrbFeature{

public:
	OrbFeature(const rgbd_slam::ParameterReader& para){
		
		int  features  = para. getData < int >( "orb_features" ); 
        float    scale = para. getData < float >( "orb_scale" ); 
        int      level = para. getData < int >( "orb_levels" ); 
        int      ini   = para. getData < int >( "orb_iniThFAST" ); 
        int      min   = para. getData < int >( "orb_minThFAST" ); 
        extractor = make_shared<ORB_SLAM2::ORBextractor>( features, scale, level, ini, min ); 
        matcher =  cv::DescriptorMatcher::create ( "BruteForce-Hamming" ); 
        knn_match_ratio = para. getData < double >( "knn_match_ratio" ); 
	}

void detectFeatures( rgbd_slam::RGBDFrame::Ptr& frame ) const
    {
        cv::Mat gray = frame->rgb;

        if (frame->rgb.channels() == 3)
        {
            // The BGR image
            cv::cvtColor( frame->rgb, gray, cv::COLOR_BGR2GRAY );
        }

        vector<cv::KeyPoint> kps;
        cv::Mat desps;
        (*extractor) ( gray, cv::Mat(), kps, desps);
        for ( size_t i=0; i<kps.size(); i++ )
        {
            rgbd_slam::Feature feature;
            feature.keypoint = kps[i];
            feature.descriptor = desps.row(i).clone();
            feature.position = frame->project2dTo3d( kps[i].pt.x, kps[i].pt.y );
            frame->features.push_back( feature );
        }
    }


	vector<cv::DMatch>  match( const rgbd_slam::RGBDFrame::Ptr& frame1, const rgbd_slam::RGBDFrame::Ptr& frame2 ) const;


protected:
    shared_ptr<ORB_SLAM2::ORBextractor> extractor;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    double knn_match_ratio =0.8;





};
} 
