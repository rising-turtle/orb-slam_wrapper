#include "img2pc.h"

using namespace std; 

void generateColorPC( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pc, cv::Mat rgb, cv::Mat depth, CamModel& cam)
{
    // pointcloud_type::Ptr new_pc(new pointcloud_type); 
    // PointCloud::Ptr cloud ( new PointCloud );
    // 遍历深度图
    
  double camera_scale = 0.001; // this should be parameterized in CamModel
    double dx, dy, dz; 
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            // PointT p;
            // point_type p; 
            pcl::PointXYZRGBA p; 

            // 计算这个点的空间坐标
            p.z = double(d) * camera_scale;
            dz = p.z; 

            cam.convertUVZ2XYZ(n, m, dz, dx, dy, dz); 
            
            p.x = dx; p.y = dy; p.z = dz; 

            // p.x = (n - camera_cx) * p.z / camera_fx;
            // p.y = (m - camera_cy) * p.z / camera_fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            pc->points.push_back( p );
        }
    // 设置并保存点云
    pc->height = 1; // depth.rows; // 1;
    pc->width = pc->points.size(); // depth.cols; // pc->points.size();
return ; 
}

