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
