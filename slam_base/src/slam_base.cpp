#include"slam_base.h"

PointCloud::Ptr slam_base::point2d_to_3d(const cv::Mat&rgb, const cv::Mat& depth){

	PointCloud::Ptr cloud ( new PointCloud );
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            ushort d = depth.ptr<ushort>(m)[n];
            if (d == 0)
                continue;
            PointT p;

            p.z = double(d) / slam_base::camera_factor;
            p.x = (n - slam_base::camera_cx) * p.z / slam_base::camera_fx;
            p.y = (m - slam_base::camera_cy) * p.z / slam_base::camera_fy;
            
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
