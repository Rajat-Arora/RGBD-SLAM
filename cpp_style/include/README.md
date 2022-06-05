common.h : namespace "rgbd_slam"
	struct CAMERA_INTRINSIC_PARAMETERS : To store camera internal parameters.
	#define different colors for printing to liux terminal

parameter_reader.h : namespace "rgbd_slam" Class: ParameteReader
 In the constructor of this class, pass in the path where the parameter file is located.
 As getData has template reader it can be used as double d = parameterReader.getData<double>("d");
 map<string, string> data; To store name and value of parameter.
 getCamera() Function is used to return the struct of camera paremeters.



rgbdframe.h: namespace "rgbd_slam" Class: RGBDFrame 
The basic unit of program operation is Frame, and the data we read from the dataset is also in Frame
The pointer of this class as shared_ptr is defined, and try to use this pointer to manage objects of this class in the future, so as to avoid some variable scope problems. Moreover, the smart pointer can delete itself, which is not prone to problems.
Everything related to this Frame is put in the members of this class, such as images, features, corresponding camera models, BoW parameters, etc. 
Finally, project2dTo3dLocal can convert a pixel coordinate to a 3D coordinate under the current Frame. Of course, the premise is that the depth point is detected in the depth map.

class FrameReader
Its constructor needs to have a reference to parameterReader, because we need to go to the parameter file to query the directory where the data is located. 
If the query is successful, it will do some initialization work, and then the external class can get the image of the next frame through the next() function.


feature.h : class Feature
	keypoint, descriptor, position and observe_frequency.


orb.h : class OrbFeature
	Uses the feature part encapsulated from orbslam2.
	constructor intializes the object of extractor and matcher.
method: detectFeatures-> detects features and store in the frame struct
method: match-> i/p: 2 frames and outputs vector<cv::DMatch> based on brute force knn matching.

pnp.h : struct PNP_INFORMATION: contains relative transformation matrix T and number of Inliers and Matches
		class PnPSolver: methods solvePnP: input 2d  curr frame and 3d points
								 solvePnPFrame: input frame1 and frame2 and outputs the transformation matrix.

convertor.h : class Convetor inherets Convertor class from g2o.
	functions:  toSE3Quat: converts from T matrix to SE3Quat for g2o.
				toCvCameraMatrix: changes from camera param struct to camera matrix
				cvRT2EigenIsometry: Changes from rvec,tvec to T matrix.
