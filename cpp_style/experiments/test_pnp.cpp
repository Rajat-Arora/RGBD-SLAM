#include "pnp.h"
#include "common_headers.h"

using namespace std;
using namespace rgbd_slam;

int main()
{
	
    cout<<"running test pnp"<<endl;
    ParameterReader para;
    FrameReader frameReader( para );
    OrbFeature orb(para);
    PnPSolver pnp(para, orb);

	RGBDFrame::Ptr refFrame = frameReader.next();
    orb.detectFeatures( refFrame );
    // Speed is the relative transformation of frame from t-2 to t-1. Used as in pnp.solveLazy to provide an 
 	//estimate for relative transformation which is taken as init_transform = frame1->T_f_w.inverse() * frame2->T_f_w;
	Eigen::Isometry3d speed = Eigen::Isometry3d::Identity();

   
	 while (RGBDFrame::Ptr currFrame = frameReader.next() )
    {
        cout<<"*************************************"<<endl;
	//	boost::timer timer;
	    
//		cout<<currFrame->rgb.size();	
		if ( currFrame == nullptr )
        {
            break;
        }
		
		currFrame->T_f_w = speed * refFrame->T_f_w ;
        orb.detectFeatures( currFrame );
	
		PNP_INFORMATION info;
   	    bool result = pnp.solvePnPFrame( refFrame, currFrame, info, true );
		
		if ( result == false )
        {
            cout<<"pnp failed"<<endl;
            refFrame = currFrame;
            cv::waitKey(0);
        }
	    else
		{
			//Update the transformation from estimated(speed*refFramr->T_f_w) from previous frame to the estimated by g2o.
			currFrame->T_f_w = info.T * refFrame->T_f_w;
            cout<<"result.T="<<endl;
            cout<<info.T.matrix()<<endl;
            cout<<"current = "<< endl << currFrame->T_f_w.matrix() << endl;
            speed = info.T;
            refFrame = currFrame;
            cv::waitKey(1);
		}	
	
	}

}
