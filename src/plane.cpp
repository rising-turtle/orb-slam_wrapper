#include "plane.h"
#include "keyframe.h"

using namespace std; 
using namespace ORB_SLAM2; 

long int CPlaneF::sNext = 0; 

CPlaneF::CPlaneF(const cv::Mat& nv_d, KeyFrame* pRefKF)
{
  mpKF = pRefKF; 
  mId = sNext++;
  // mpFrame = NULL;  // not sure now 
  nv_d.copyTo(mParam); 
  mnObs = 1;
  mbBad = false; 
}

CPlaneF::CPlaneF(const cv::Mat& nv_d) // , Frame* pFrame)
{
  nv_d.copyTo(mParam); 
  // mpFrame = pFrame; 
  mId = -1; // not assigned yet 
  mpKF = 0; 
  mnObs = 1;
  mbBad = false; 
}

CPlaneF::~CPlaneF(){}

cv::Mat CPlaneF::getParam()
{
  unique_lock<mutex> lock(mMutexPos); 
  return mParam; 
}

float CPlaneF::dis2Plane(float px, float py, float pz)
{
  unique_lock<mutex> lock(mMutexPos); 
  float ret = mParam.at<float>(0,0)*px + mParam.at<float>(1,0)*py + mParam.at<float>(2,0)*pz + mParam.at<float>(3,0); 
  return ret; 
}

void CPlaneF::getParam(Eigen::Vector4d& v)
{
  unique_lock<mutex> lock(mMutexPos); 
  v << mParam.at<float>(0,0), mParam.at<float>(1,0), mParam.at<float>(2,0), mParam.at<float>(3,0); 
  return ;
}

std::map<KeyFrame*, size_t> CPlaneF::getObservations()
{
  unique_lock<mutex> lock(mMutexFeatures); 
  return mmObs; 
}

void CPlaneF::addObservation(KeyFrame* pKF, size_t idx)
{
  unique_lock<mutex> lock(mMutexFeatures); 
  if(mmObs.count(pKF))
    return; 
  mmObs[pKF] = idx; 
  mnObs++; 
}

bool CPlaneF::isBad()
{
  unique_lock<mutex> lock(mMutexFeatures); 
  unique_lock<mutex> lock2(mMutexPos); 
  return mbBad; 
}

void CPlaneF::setBadFlag()
{
  map<KeyFrame*, size_t> obs; 
  {
    unique_lock<mutex> lock(mMutexFeatures); 
    unique_lock<mutex> lock2(mMutexPos); 
    mbBad = true; 
    obs = mmObs; 
    mmObs.clear(); 
  }
  
  for(map<KeyFrame*, size_t>::iterator it = obs.begin(), iend = obs.end(); it!=iend; ++it)
  {
    KeyFrame* pKF = it->first; 
    ((CKeyFrame*)pKF)->erasePlaneObs(it->second);
  } 
}

CPlaneF* CPlaneF::transformToWorld(KeyFrame* pKF)
{
  CPlaneF* rF = new CPlaneF(mParam, pKF); 
  // mpKF = pRefKF; 
  // mId = sNext++;
  
  Eigen::Vector4d wP; 
  rF->transform(pKF->GetPose(), wP); 
  rF->setParam(wP); 
  return rF; 
}

int CPlaneF::getIndexKF(ORB_SLAM2::KeyFrame* pKF)
{
  unique_lock<mutex> lock(mMutexFeatures); 
  if(mmObs.count(pKF))
    return mmObs[pKF]; 
  
  return -1;
}

void CPlaneF::eraseObservation(KeyFrame* pKF)
{
  bool bBad = false; 
  {
    unique_lock<mutex> lock(mMutexFeatures); 
    if(mmObs.count(pKF))
    {
      int idx = mmObs[pKF]; 
      mnObs-- ;
      mmObs.erase(pKF); 
      
      if(mnObs <= 2) bBad = true; 
    }
  }

  if(bBad)
    setBadFlag(); 
}

void CPlaneF::flipParam()
{
  unique_lock<mutex> lock(mMutexPos); 
  mParam = mParam * -1;
}
void CPlaneF::setParam(Eigen::Vector4d& v)
{
  unique_lock<mutex> lock(mMutexPos); 
  mParam.at<float>(0,0) = v(0); 
  mParam.at<float>(1,0) = v(1); 
  mParam.at<float>(2,0) = v(2); 
  mParam.at<float>(3,0) = v(3); 
}

// this function acts not weired, since we needs cvT.inverse() rather than cvT, 
// this happens since the Frame class only has Tcw , do not has Twc
void CPlaneF::transform(cv::Mat cvT, Eigen::Vector4d& nw)
{
  Eigen::Matrix<double,3,3> R;
  R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
    cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
    cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

  Eigen::Matrix<double, 3, 3> Rt = R.transpose(); 

  Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));
  t = -Rt*t;

  Eigen::Vector4d nc; 
  {
    unique_lock<mutex> lock(mMutexPos); 
    nc << mParam.at<float>(0,0), mParam.at<float>(1,0), mParam.at<float>(2,0), mParam.at<float>(3,0); 
  }
  // Eigen::Vector4d nw; 
  nw.head<3>() = Rt * nc.head<3>(); 
  nw(3) = nc(3) - t.dot(nw.head<3>()); 
  double n = nw.head<3>().norm();
  nw = nw * (1./n); 
  return ; 
}


