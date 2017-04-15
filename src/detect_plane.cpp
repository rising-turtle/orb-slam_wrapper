
#include "detect_plane.h"
#include "img2pc.h"
#include "plane_set.h"

CExtractPlane::CExtractPlane(CamModel& cam) : 
mCam(cam)
{}
CExtractPlane::~CExtractPlane(){}

std::vector<CPlaneF*> CExtractPlane::extractPlanes(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pPC, double squared_dis_threshold)
{
  std::vector<CPlaneF*> ret; 

  if(pPC->points.size() < 20000)
  {
    return ret;
  }

  CPlaneSet pS; 
  int np = pS.extractPlanes(pPC, squared_dis_threshold); 
  if(np <= 0)  // no plane detected 
  {
    return ret; 
  }

  // create new plane features 
  cv::Mat nv_d = cv::Mat::zeros(4, 1, CV_32F); 
  for(int i=0; i<np; i++)
  {
    CPlane* p = pS.v_plane_set_[i]; 
    nv_d.at<float>(0,0) =  p->nx_; 
    nv_d.at<float>(1,0) =  p->ny_; 
    nv_d.at<float>(2,0) =  p->nz_; 
    nv_d.at<float>(3,0) =  p->d1_; 
    CPlaneF* p3D = new CPlaneF(nv_d); 
    ret.push_back(p3D); 
  }
  
  return ret; 
}

std::vector<CPlaneF*> CExtractPlane::extractPlanes(double timestamp, double squared_dis_threshold)
{
  std::vector<CPlaneF*> ret; 
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPC(new pcl::PointCloud<pcl::PointXYZRGBA>); 
  bool bPC = extractPointCloud(timestamp, pPC); 
  if(!bPC)
  {
    cout <<"detect_plane.cpp: failed to extract PC at timestamp = "<<timestamp<<endl;
    return ret; 
  }
  
  return extractPlanes(pPC, squared_dis_threshold); 
}

bool CExtractPlane::extractPointCloud(double timestamp, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pPC)
{
  int id = findCurrentFrameId(timestamp); 
  if(id == -1) return false; 

  string rgbF = mDir + "/" + mvstrImageFilenamesRGB[id]; 
  string dptF = mDir + "/" + mvstrImageFilenamesD[id]; 
  
  cv::Mat imRGB = cv::imread(rgbF,CV_LOAD_IMAGE_UNCHANGED);
  cv::Mat imD = cv::imread(dptF,CV_LOAD_IMAGE_UNCHANGED);

  if(imRGB.empty() || imD.empty())
  {
    cerr <<"detect_plane.cpp: imRGB or imD is empty"<<endl; 
    return false; 
  }
  
  // generate point cloud 
  generateColorPC(pPC, imRGB, imD, mCam); 

  return true; 
}

void CExtractPlane::setData(string Dir, vector<double>& vTimeStamps, vector<string>& vFRGB, vector<string>& vFDPT )
{
  mDir = Dir; 
  mvTimestamps = vTimeStamps; 
  mvstrImageFilenamesRGB = vFRGB; 
  mvstrImageFilenamesD = vFDPT; 
}

int CExtractPlane::findCurrentFrameId(double timestamp)
{
  for(int i=0; i<mvTimestamps.size(); i++)
  {
    if(mvTimestamps[i] > timestamp) 
    {
      cout <<" Has not found timestamp = "<<timestamp<< " in mvTimestamps"<<endl;
      return -1; 
    }
    if(mvTimestamps[i] == timestamp)
      return i; 
  }
  return -1;
}

