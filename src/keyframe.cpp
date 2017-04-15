#include "keyframe.h"
#include <Eigen/Core>

using namespace ORB_SLAM2; 
using namespace std; 

CKeyFrame::CKeyFrame(Frame& F, Map* pMap, KeyFrameDatabase* pKFDB): 
  KeyFrame(F, pMap, pKFDB)
{
  // 

}

CKeyFrame::~CKeyFrame()
{
  for(size_t i=0; i<mvpCurrPF.size(); i++)
  {
    if(mvpCurrPF[i])
      delete mvpCurrPF[i]; 
  }
  mvpCurrPF.clear(); 
}

std::vector<CPlaneF*> CKeyFrame::getPlaneFeatures()
{
  return mvpPlaneF; 
}

CPlaneF* CKeyFrame::getPlaneObservationAt(int idx)
{
  if(idx < 0 || idx >= mvpCurrPF.size())
  {
    cerr <<" keyframe.cpp: what? idx = "<<idx<<endl;
  }
  assert(idx >=0 && idx < mvpCurrPF.size()); 
  return mvpCurrPF[idx]; 
}

CPlaneF* CKeyFrame::getPlaneFeatureAt(int idx)
{
  assert(idx >=0 && idx < mvpPlaneF.size()); 
  return mvpPlaneF[idx]; 
}

void CKeyFrame::erasePlaneObs(size_t idx)
{
  assert(idx >=0 && idx <mvpPlaneF.size()); 
  mvpPlaneF[idx] = static_cast<CPlaneF*>(0); 
  return ;
}

void CKeyFrame::erasePlaneObs(CPlaneF* p)
{
  int idx = p->getIndexKF(this); 
  return erasePlaneObs(idx); 
}
