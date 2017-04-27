#include "system.h"
#include "track.h"
#include "cam_model.h"
#include "local_map.h"
#include "Converter.h"

using namespace ORB_SLAM2; 

CSystem::CSystem(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer) : 
  System(strVocFile, strSettingsFile, sensor, bUseViewer)
{
  // handle tracker 
  mpTrack = new CTrack(*mpTracker); 
  mpTracker = mpTrack; 

  // close loop close thread 
  mpLoopCloser->RequestFinish();

  // close local mapping thread 
  mpLocalMapper->RequestFinish(); 
  while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished())
  {
    usleep(1000); 
  }
  
  // restart local mapping thread 
  mpLocalMap = new CLocalMap(mpMap, mSensor == MONOCULAR); 
  mpLocalMapper = mpLocalMap;
  
  mptLocalMapping = new thread(&CLocalMap::Run, mpLocalMap); 
  // mptLocalMapping = new thread(&LocalMapping::Run, mpLocalMapper); 

  // reset pointer 
  mpTracker->SetLocalMapper(mpLocalMapper); 
  mpLocalMapper->SetTracker(mpTracker); 
  mpLocalMapper->SetLoopCloser(mpLoopCloser); 
  mpLoopCloser->SetLocalMapper(mpLocalMapper); 
}

CSystem::~CSystem(){}

cv::Mat CSystem::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{ 
  if(mSensor!=RGBD)
  {
    cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
    exit(-1);
  } 
  cv::Mat Tcw = mpTrack->GrabImageRGBD(im, depthmap, timestamp); 
  // convertPos(Tcw); 
  return Tcw; 
  // return System::TrackRGBD(im, depthmap, timestamp); 
}

void CSystem::convertPos(cv::Mat pos)
{
  vector<float> q; 
  q = ORB_SLAM2::Converter::toQuaternion(pos); 
  
  float* p = mpos; 
  p[0] = pos.at<float>(0,3); p[1] = pos.at<float>(1,3); p[2] = pos.at<float>(2,3);
  p[3] = q[0]; p[4] = q[1]; p[5] = q[2]; p[6] = q[3];
  return; 
}

void CSystem::enablePlanes(string imgDir, vector<string>& vRGBf, vector<string>& vDPTf, vector<double>& Timestamp, CamModel& cam)
{
  mpTrack->enablePlanes(imgDir, vRGBf, vDPTf, Timestamp, cam);
  mpLocalMap->mbUsePlane = true; 
  return; 
}


bool CSystem::isNewKF()
{
  return mpTrack->isNewKF(); 
}

void CSystem::saveMapPoints(string filename)
{
  ofstream ouf(filename.c_str()); 
  if(!ouf.is_open()) 
  {
    cerr <<" system.cpp failed to open file "<<filename<<endl; 
    return;
  }
  cout <<endl<<" saving MapPoints to "<<filename<<endl; 
  vector<MapPoint*> vpMP = mpMap->GetAllMapPoints(); 
  
  int cnt = 0;
  for(int i=0; i<vpMP.size(); i++)
  {
    MapPoint* pMP = vpMP[i]; 
    if(pMP && !pMP->isBad())
    {
      cv::Mat pos = pMP->GetWorldPos(); 
      ouf << pos.at<float>(0)<<" "<<pos.at<float>(1)<<" "<<pos.at<float>(2)<<endl;
      ++cnt; 
    }
  }
  
  ouf.close(); 
  cout <<"system.cpp: succeed to save "<<cnt<<" MapPoints!"<<endl; 
  return ;
}

