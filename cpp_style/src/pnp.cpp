#include "convertor.h"
#include "pnp.h"
#include "orb.h"

bool rgbd_slam::PnPSolver::solvePnP( const vector<cv::Point2f>& img, const vector<cv::Point3f>& obj, const rgbd_slam::CAMERA_INTRINSIC_PARAMETERS& camera, vector<int>& inliersIndex, Eigen::Isometry3d& transform ){
	
	g2o::SparseOptimizer optimizer; 
	g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>(); //6 is dimension of camera pose and 3 is of feature position.
	g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr ); //Using LM Algorithm for solving non linear optimization
	optimizer.setAlgorithm( solver );
	
	//SE3 Vertex parameterized internally with a transformation matrix and externally with its exponential map.
	g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
	vSE3->setEstimate( rgbd_slam::Converter::toSE3Quat( Eigen::Isometry3d::Identity()) );
	vSE3->setFixed( false ); //It will be included in optimization step as it is not set fixed.
	vSE3->setId(0);
    optimizer.addVertex( vSE3 );

	//unary edge to optimize only the camera pose
	// This edge has only one endpoint, which is se3 itself 
    // Load it with a vectors first, then use it later

	vector<g2o::EdgeSE3ProjectXYZOnlyPose*> edges;
	
	const float delta = sqrt(5.991);
	vector<bool> inliers( img.size(), true );
    int good = 0;
	
	for ( size_t i=0; i<obj.size(); i++ )
    {
        if (obj[i] == cv::Point3f(0,0,0))
        {
            inliers[i] = false;
            continue;
        }
        good++;	
	
		g2o::EdgeSE3ProjectXYZOnlyPose * edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		edge->setMeasurement( Eigen::Vector2d(img[i].x, img[i].y) );
		edge->fx = camera.fx;
        edge->fy = camera.fy;
        edge->cx = camera.cx;
        edge->cy = camera.cy;
        edge->Xw = Eigen::Vector3d( obj[i].x, obj[i].y, obj[i].z );
		edge->setInformation( Eigen::Matrix2d::Identity()*1 );
		
		g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
		edge->setRobustKernel( rk );
		rk->setDelta( delta );
        optimizer.addEdge( edge );
        edge->setId( i );
        edges.push_back(edge);
	}


    // Use g2o to determine inliers 
    // A total of four iterations are performed, each iteration is ten times
	for ( size_t  it= 0 ; it< 4 ; it++) 
    {	
		vSE3->setEstimate( rgbd_slam::Converter::toSE3Quat( transform ) );
        optimizer.initializeOptimization(0);
        optimizer.optimize( 10 ); 
	
      for ( size_t i=0; i<edges.size(); i++ ){
		g2o::EdgeSE3ProjectXYZOnlyPose* e = edges[i];
            if ( inliers[ e->id() ] == true )
            {
                e->computeError();
            }
			if ( e->chi2() > 5.991 ) // If chi2 is greater than a threshhold it is an outlier.
            { 
                inliers[ e->id() ] = false;
                e->setLevel(1);
                good -- ;
            }
			else
            {
                inliers[i] = true;
                e->setLevel(0);
            }
			// After removing the edge with larger error, there is no need to use robust kernel anymore 
            if (it== 2 ) 
			e-> setRobustKernel ( nullptr ); 
      }
	      if (good < 5)
            break;
    }
     			

	for ( size_t i=0; i<inliers.size(); i++ )
    {
        if ( inliers[i] )
        {
            inliersIndex.push_back(i); //Store index of inliers.
        }
    }
	
	g2o::VertexSE3Expmap* vSE_recov = dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(0));
    g2o::SE3Quat    se3_recov = vSE_recov->estimate();

    transform = Eigen::Isometry3d( se3_recov );

    if (inliers.size() > min_inliers)
        return true;
    return false;
}


bool rgbd_slam::PnPSolver::solvePnPFrame( const rgbd_slam::RGBDFrame::Ptr & frame1, const rgbd_slam::RGBDFrame::Ptr& frame2, rgbd_slam::PNP_INFORMATION& pnp_information, bool drawMatches){

	vector<cv::DMatch>  matches = orb.match( frame1, frame2 );
    if ( matches.size() <= min_match )
        return false;
    vector<cv::Point3f> obj;
    vector<cv::Point2f> img;

    vector<cv::DMatch>  validMatches;

    for (auto m:matches)
    {
        cv::Point3f pObj = frame1->features[m.queryIdx].position;
        if (pObj == cv::Point3f(0,0,0))
            continue;
        if (drawMatches)
        {
            validMatches.push_back(m);
        }

	    obj.push_back( pObj );
        img.push_back( frame2->features[m.trainIdx].keypoint.pt );
    }

	vector<int> inliersIndex;
    Eigen::Isometry3d   init_transform = frame1->T_f_w.inverse() * frame2->T_f_w;
    //cout<<"init transform = "<<init_transform.matrix()<<endl;
    bool b = solvePnP( img, obj, frame1->camera, inliersIndex, init_transform );

    pnp_information.numFeatureMatches = img.size();
    pnp_information.numInliers = inliersIndex.size();
    pnp_information.T = init_transform;

	if (drawMatches == true && b==true)
    {
        vector<cv::DMatch> inlierMatches;
        for ( int index:inliersIndex )
            inlierMatches.push_back( validMatches[index] );
        cv::Mat out;
        cv::drawMatches(frame1->rgb, frame1->getAllKeypoints(),
                        frame2->rgb, frame2->getAllKeypoints(),
                        inlierMatches, out );
        cv::imshow( "inlier matches", out );
    }

    if ( pnp_information.numInliers < min_inliers )
    {
        return false;
    }
    return true;

}



