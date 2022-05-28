#pragma once

#include "common_headers.h"
#include "parameter_reader.h"
#include "feature.h"
#include "utils.h"

#include"Thirdparty/DBoW2/DBoW2/FORB.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <mutex>

/*
Since Pose may be accessed by several threads at the same time, it needs to be locked. 
A lock allows you to force multiple threads to access a resource one at a time, 
rather than all of them trying to access the resource simultaneously
*/

namespace rgbd_slam{

class RGBDFrame{

public:
	typedef shared_ptr<RGBDFrame> Ptr;
	
	//----------------------------------member variables---------------------------------------------------

	int id=-1; //means frame does not exsist
	cv::Mat rgb, depth; //To store rgb image and depth image
	
	Eigen::Isometry3d T_f_w = Eigen::Isometry3d::Identity(); //Transform from current frame to world frame

	std::mutex mutexT;
	
	vector<rgbd_slam::Feature> features; //Vector of features

    CAMERA_INTRINSIC_PARAMETERS camera; 	

    DBoW2::BowVector bowVec; //vector to detect loopback. Stores image as bag of visual words.

    pcl::PointCloud<pcl::PointXYZRGBA>:: Ptr pointcloud = nullptr; //Point clod corresponding to image.
	
	//-------------------------------------------------------------------------------------------------------

public:
	
	//---------------------------Member functions---------------------------------------------------

	//Project pixels to local 3d coordinates.
	
	cv::Point3f project2dTo3d(int u, int v) const
    {
        if (depth.data == nullptr)
            return cv::Point3f(0,0,0);
        ushort d = depth.ptr<ushort>(v)[u];
        if (d == 0)
            return cv::Point3f(0,0,0);
        cv::Point3f p;
        p.z = double( d ) / camera.scale;
        p.x = ( u - camera.cx) * p.z / camera.fx;
        p.y = ( v - camera.cy) * p.z / camera.fy;
        return p;
    } 

	//Get Mat of all descriptors of festures.
	cv::Mat getAllDescriptors() const
    {
        cv::Mat desp;
        for ( size_t i=0; i<features.size(); i++ )
        {
            desp.push_back( features[i].descriptor );
        }
        return desp;
    }

	//Get vector of all descriptors	
	vector<cv::Mat> getAllDescriptorsVec() const
    {
        vector<cv::Mat> desp;
        for ( auto f:features )
        {
            desp.push_back( f.descriptor );
        }
        return desp;
    }

	//Get vector of all keypoints
	vector<cv::KeyPoint> getAllKeypoints() const
    {
        vector<cv::KeyPoint> kps;
        for ( auto f:features )
        {
            kps.push_back( f.keypoint );
        }
        return kps;
   }

	void setTransform( const Eigen::Isometry3d& T )
    {
        std::unique_lock<std::mutex> lck(mutexT);
        T_f_w = T;
    }
    
    
	Eigen::Isometry3d getTransform()  
    {
        std::unique_lock<std::mutex> lck(mutexT);
        return T_f_w;
    }
};



class FrameReader{

public:
    enum DATASET
    {
        NYUD=0,
        TUM=1,
    };

protected:
	//Member Variables

    DATASET dataset_type = TUM;

    int currentIndex =0;
    int start_index  =0;
    vector<string> rgbFiles, depthFiles;
    string  dataset_dir;

    const ParameterReader& parameterReader; //Parameter reader obj
    CAMERA_INTRINSIC_PARAMETERS camera; //Camera parameters struct obj

public:

	//-----------------------Member Functions---------------------------------

	FrameReader(rgbd_slam::ParameterReader& para, const DATASET& dataset_type = TUM)
    	: parameterReader(para)
	{
		this->dataset_type = dataset_type;

        switch( dataset_type )
        {
        case NYUD:
            //Implement later
            break;
        case TUM:
            init_tum( para );
            break;
        }

        camera = para.getCamera();
	}

	RGBDFrame::Ptr next(); //Gives a pointer to next frame by changing the index

    void reset()
    {
        currentIndex = start_index;
    }	

	// get frame by index
    RGBDFrame::Ptr   get( int index )
    {
        if (index < 0 || index >= rgbFiles.size() )
            return nullptr;
        currentIndex = index;
        return next();
    }

protected:
	void init_tum( ParameterReader& para );
	
};

}
