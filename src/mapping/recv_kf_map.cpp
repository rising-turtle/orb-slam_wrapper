/*
 *  Apr. 19 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  recv data from keyframes and send them to lsd_viewer for mapping 
 *
 * */

#include <ros/ros.h>
#include "lsd_slam_viewer/keyframeMsg.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <mutex>

#include <sensor_msgs/image_encodings.h> 
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "cam_model.h"
#include <opencv2/core/core.hpp>
#include "util/SophusUtil.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std; 

// global 
cv::Mat gRGB; 
cv::Mat gDpt; 
cv::Mat gPos;
bool gNew = false; 
std::mutex gMutex; 

struct Traj
{
  double timestamp; 
  float x, y, z, qx, qy, qz, qw; 
};

struct InputPointDense
{
  float idepth;
  float idepth_var;
  unsigned char color[4];
};

void generateKFMsg(Traj& Ti, cv::Mat rgb, cv::Mat dpt, CamModel& cam, ros::Publisher* pPub); 
Traj computeT(cv::Mat Pos); 

void ImageGrabber(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::ImageConstPtr& pos)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;
    cv_bridge::CvImageConstPtr cv_ptrPos;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB, sensor_msgs::image_encodings::BGR8);
        cv_ptrD = cv_bridge::toCvShare(msgD, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptrPos = cv_bridge::toCvShare(pos, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    {
      unique_lock<mutex> lock(gMutex); 
      gRGB = cv_ptrRGB->image.clone(); 
      gDpt = cv_ptrD->image.clone(); 
      gPos = cv_ptrPos->image.clone(); 
      gNew = true; 
    }
    return; 
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "kf_map"); 
  ros::NodeHandle n; 

  int q = 7; 
  // publish to lsd_viewer 
  std::string kf_channel = n.resolveName("lsd_slam/keyframes");
  ros::Publisher kf_publisher = n.advertise<lsd_slam_viewer::keyframeMsg>(kf_channel, q); 
  
  // std::string gkf_channel = n.resolveName("lsd_slam/graph"); 
  // ros::Publisher gkf_publisher = n.advertise<lsd_slam_viewer::keyframeGraphMsg>(gkf_channel, q); 

  // subsribe keyframes from robocane 
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, "/robocane/kf_rgb", q);
  message_filters::Subscriber<sensor_msgs::Image> dpt_sub(n, "/robocane/kf_dpt", q);
  message_filters::Subscriber<sensor_msgs::Image> pos_sub(n, "/robocane/kf_pos", q); 
    
  // message_filters::Subscriber<sensor_msgs::Image> ir1_sub(nh, "/camera/ir/image_raw", q); 
  //  message_filters::Subscriber<sensor_msgs::Image> ir2_sub(nh, "/camera/ir2/image_raw", q); 
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync_pol;
  //   message_filters::Synchronizer<sync_pol> sync(sync_pol(q), rgb_sub, depth_sub, ir1_sub, ir2_sub, rgb_cam);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(q), rgb_sub, dpt_sub, pos_sub);
  sync.registerCallback(boost::bind(&ImageGrabber,_1,_2, _3));

  // cam model for r200
  CamModel cam(629.315, 635.483, 308.579, 239.042, -0.0845005, 0.0486205);
  cam.width = 640; 
  cam.height = 480; 
  
  cv::Mat imRGB, imD; 
  cv::Mat lRGB, lDpt, lPos; 
  bool lNew; 
  sleep(2); // wait for the mapping program start
  while(ros::ok())
  {
      {
        lNew = false; 
        unique_lock<mutex> lock(gMutex); 
        if(gNew)
        {
          lRGB = gRGB.clone(); 
          lDpt = gDpt.clone(); 
          lPos = gPos.clone(); 
          gNew = false; 
          lNew = true; 
        }
      }
      if(lNew)
      {
        Traj Ti = computeT(lPos); 
        generateKFMsg(Ti, lRGB, lDpt, cam, &kf_publisher);

        lNew = false; 
      }
      ros::spinOnce();
      usleep(1000); 
  }
  ros::shutdown(); 

  return 0; 
}

Traj computeT(cv::Mat p)
{
  Eigen::Matrix<double, 3, 3> R; 
  R << p.at<float>(0,0), p.at<float>(0,1), p.at<float>(0,2), 
       p.at<float>(1,0), p.at<float>(1,1), p.at<float>(1,2), 
       p.at<float>(2,0), p.at<float>(2,1), p.at<float>(2,2); 
  Eigen::Quaterniond q(R); 
  Traj Ti;
  Ti.timestamp = 10000; // ros::Time::Now().toSec(); 
  Ti.x = p.at<float>(0,3); Ti.y = p.at<float>(1,3); Ti.z = p.at<float>(2,3); 
  Ti.qx = q.x(); Ti.qy = q.y(); Ti.qz = q.z(); Ti.qw = q.w(); 
  return Ti; 
}

void generateKFMsg(Traj& Ti, cv::Mat rgb, cv::Mat dpt, CamModel& cam, ros::Publisher* pPub)
{
  static int sid = 1; 
  lsd_slam_viewer::keyframeMsg fMsg; 
  fMsg.id = sid ++ ;
  fMsg.time = Ti.timestamp; 
  fMsg.isKeyframe = true; 
  fMsg.fx = cam.fx; 
  fMsg.fy = cam.fy; 
  fMsg.cx = cam.cx; 
  fMsg.cy = cam.cy; 
  fMsg.width = cam.width; 
  fMsg.height = cam.height; 
  
  // cout <<"Let's build fMsg!"<<endl; 

  // float p[7];  
  // p[0] = Ti.x; p[1] = Ti.y; p[2] = Ti.z; p[3] = Ti.qx; p[4] = Ti.qy; p[5] = Ti.qz; p[6] = Ti.qw; 
  Eigen::Quaternion<double> qua(Ti.qw, Ti.qx, Ti.qy, Ti.qz); 
  Eigen::Matrix<double, 3, 1> point; point(0, 0) = Ti.x; point(1, 0) = Ti.y; point(2, 0) = Ti.z; 
  Sophus::SE3d se3p(qua, point); 
  Sophus::Sim3d sim3p; 
  sim3p = lsd_slam::sim3FromSE3(se3p, 1); 
  memcpy(fMsg.camToWorld.data(), sim3p.cast<float>().data(), sizeof(float)*7); 

  // keep it for send a graph connection 
  // GFPose p; 
  // p.id = fMsg.id; 
  // memcpy(p.camToWorld, sim3p.cast<float>().data(), sizeof(float)*7); 
  // vGPs.push_back(p); 

  fMsg.pointcloud.resize(fMsg.width * fMsg.height * sizeof(InputPointDense)); 

  InputPointDense* pc = (InputPointDense*)fMsg.pointcloud.data(); 
  float invd; 
  // cout <<"before build pointcloud!"<<endl; 
  for(int m = 0; m < fMsg.height; m++)
  for(int n = 0; n < fMsg.width; n++)
  {
    int idx  = m * fMsg.width + n; 
    // ushort d = dpt.ptr<ushort>(m)[n]; 
    float d = dpt.ptr<float>(m)[n]; 
    if(d == 0) 
      invd = 0; 
    else
      invd = 1./(float)(d); 
    pc[idx].idepth = invd; 
    pc[idx].idepth_var = 0.00001; // fake one 
    pc[idx].color[0] = rgb.ptr<uchar>(m)[n*3+2]; 
    pc[idx].color[1] = rgb.ptr<uchar>(m)[n*3+1]; 
    pc[idx].color[2] = rgb.ptr<uchar>(m)[n*3+0]; 
    pc[idx].color[3] = 1; 
  }
  // cout <<"after build point cloud"<<endl;
  // return fMsg; 
  pPub->publish(fMsg); 
  return ; 
}

