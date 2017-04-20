/*
 *  Apr. 10 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  broadcast keyframe and point cloud to be viewed by lsd_viewer 
 *
 * */


#include <ros/ros.h>
#include "lsd_slam_viewer/keyframeMsg.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include "cam_model.h"
#include <opencv2/core/core.hpp>
#include "util/SophusUtil.h"

using namespace std; 

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

struct GraphConstraint
{
  int from;
  int to;
  float err;
};

struct GFPose
{
  int id; 
  float camToWorld[7]; 
};

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadTrajectory(string trj_file, vector<Traj>&); 

int findTimestamp(double t, vector<double>&  vTimestamps); 

void generateKFMsg(Traj& Ti, cv::Mat rgb, cv::Mat dpt, CamModel& cam, ros::Publisher* pPub); 
void generateGKFMsg(ros::Publisher* pPub); 

vector<GFPose> vGPs; 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "orb_map"); 
  ros::NodeHandle n; 

  std::string kf_channel = n.resolveName("lsd_slam/keyframes");
  ros::Publisher kf_publisher = n.advertise<lsd_slam_viewer::keyframeMsg>(kf_channel, 7); 
  
  std::string gkf_channel = n.resolveName("lsd_slam/graph"); 
  ros::Publisher gkf_publisher = n.advertise<lsd_slam_viewer::keyframeGraphMsg>(gkf_channel, 7); 

  ros::NodeHandle np("~"); 
  
  // Retrieve paths to images
  vector<string> vstrImageFilenamesRGB;
  vector<string> vstrImageFilenamesD;
  vector<double> vTimestamps;
  string imgDir(""); 
  np.param("data_dir", imgDir, imgDir); 
  string strAssociationFilename = imgDir + "/timestamp.txt";
  
  LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
  
  // Check consistency in the number of images and depthmaps
  int nImages = vstrImageFilenamesRGB.size();
  if(vstrImageFilenamesRGB.empty())
  {
    cerr << endl << "No images found in provided path." << endl;
    return 1;
  }
  else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
  {
    cerr << endl << "Different number of images for rgb and depth." << endl;
    return 1;
  }

  // Keyframe imgs
  string kf_file(""); 
  np.param("kf_file", kf_file, kf_file); 
  vector<Traj> T; 
  LoadTrajectory(kf_file, T); 
  if(T.size() == 0)
  {
    cerr <<" No trajectory is load in "<<kf_file<<endl; 
    return 1; 
  }

  // cam model for f200
  CamModel cam(621.94176, 625.35610, 319.77554, 236.21507, 0.08011, -0.56891);
  cam.width = 640; 
  cam.height = 480; 
  
  cv::Mat imRGB, imD; 
  sleep(2); // wait for the mapping program start
  for(size_t i=0; i<T.size() && ros::ok(); ++i) // publish KF information 
  {
    Traj Ti = T[i]; 
    int id = findTimestamp(Ti.timestamp, vTimestamps); 
    if(id < 0 || id >= vTimestamps.size())
    {
      cerr <<"id = "<<id <<" not right!"<<endl; 
      break; 
    }
    // Read image and depth from file
    imRGB = cv::imread(imgDir+"/"+vstrImageFilenamesRGB[id],CV_LOAD_IMAGE_UNCHANGED);
    imD = cv::imread(imgDir+"/"+vstrImageFilenamesD[id],CV_LOAD_IMAGE_UNCHANGED); 
    
    if(imRGB.empty() || imD.empty())
    {
      ROS_ERROR("orb_map.cpp: imgRGB or imD is empty() with imgDir %s", imgDir.c_str()); 
      break; 
    }

    // generate msg and publish 
    // lsd_slam_viewer::keyframeMsg& kfMsg = generateKFMsg(Ti, imRGB, imD, cam); 
    // kf_publisher.publish(kfMsg); 
    generateKFMsg(Ti, imRGB, imD, cam, &kf_publisher);
    generateGKFMsg(&gkf_publisher);
    ros::spinOnce(); 
    usleep(3000); 
    ROS_INFO("orb_map.cpp: publish data at timestamp = %lf", Ti.timestamp); 
  }

  return 0; 
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
  GFPose p; 
  p.id = fMsg.id; 
  memcpy(p.camToWorld, sim3p.cast<float>().data(), sizeof(float)*7); 
  vGPs.push_back(p); 

  fMsg.pointcloud.resize(fMsg.width * fMsg.height * sizeof(InputPointDense)); 

  InputPointDense* pc = (InputPointDense*)fMsg.pointcloud.data(); 
  float invd; 
  // cout <<"before build pointcloud!"<<endl; 
  for(int m = 0; m < fMsg.height; m++)
  for(int n = 0; n < fMsg.width; n++)
  {
    int idx  = m * fMsg.width + n; 
    ushort d = dpt.ptr<ushort>(m)[n]; 
    if(d == 0) 
      invd = 0; 
    else
      invd = 1000./(float)(d); 
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

void generateGKFMsg(ros::Publisher* pPub)
{
  lsd_slam_viewer::keyframeGraphMsg gMsg;
  gMsg.numConstraints = vGPs.size() - 1;
  gMsg.constraintsData.resize(gMsg.numConstraints * sizeof(GraphConstraint));
  GraphConstraint* constraintData = (GraphConstraint*)gMsg.constraintsData.data(); 
  for(unsigned int i=0;i<vGPs.size()-1;i++)
  {
    constraintData[i].from = vGPs[i].id; 
    constraintData[i].to = vGPs[i+1].id; 
    constraintData[i].err = 0.01;
  }

  gMsg.numFrames = vGPs.size();
  gMsg.frameData.resize(gMsg.numFrames * sizeof(GFPose));
  GFPose* framePoseData = (GFPose*)gMsg.frameData.data();
  for(unsigned int i=0;i<vGPs.size();i++)
  {
    framePoseData[i].id = vGPs[i].id;
    memcpy(framePoseData[i].camToWorld, vGPs[i].camToWorld,sizeof(float)*7);
  }

  pPub->publish(gMsg); 
}

int findTimestamp(double t, vector<double>& vT)
{
  for(size_t i=0; i<vT.size(); ++i)
  {
    if(vT[i] == t)
      return i; 
    if(vT[i] > t)
    {
      ROS_WARN("orb_map.cpp: failed to find out t = %lf current vT[%d] = %lf", t, i, vT[i]);
      break; 
    }
  }
  
  ROS_WARN("orb_map.cpp: failed to find out t = %lf ", t);
  return -1;
}


void LoadTrajectory(string trj_file, vector<Traj>& T)
{
  ifstream inf(trj_file.c_str()); 

  string line; 
  while(!inf.eof())
  {
    getline(inf, line); 
    Traj p; 
    if(!line.empty())
    {
      stringstream ss; 
      ss << line; 
      ss >> p.timestamp >> p.x >> p.y >> p.z >> p.qx >> p.qy >> p.qz >> p.qw; 
      T.push_back(p); 
    }
  }
  
  cout <<"orb_map.cpp: succeed to load trajectory T.size() = "<<T.size()<<endl; 
  return ;
}


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
    vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
  ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  while(!fAssociation.eof())
  {
    string s;
    getline(fAssociation,s);
    if(!s.empty())
    {
      stringstream ss;
      ss << s;
      double t;
      string sRGB, sD;
      ss >> t;
      // printf("load Images line: %s\n", ss.str().c_str());
      // printf("loadImages t = %.9lf\n", t);
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
      ss >> t;
      ss >> sD;
      vstrImageFilenamesD.push_back(sD);

    }
  }
  cout <<"orb_map.cpp: succeed to load record "<<vTimestamps.size()<<endl;
}
