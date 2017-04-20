#include "local_map.h"
#include <iostream>
#include "optimizer.h"

using namespace ORB_SLAM2; 
using namespace std; 

CLocalMap::CLocalMap(Map* pMap, const float bMonocular) : 
  LocalMapping(pMap, bMonocular),
  mbUsePlane(false)
{}

CLocalMap::~CLocalMap(){} 


void CLocalMap::Run()
{
  mbFinished = false; 
  
  while(1)
  {
    // Tracking will see that Local Mapping is busy 
    SetAcceptKeyFrames(false); 

    // Check if there are keyframes in the queue 
    if(CheckNewKeyFrames())
    {
      // BoW conversion and insertion in Map 
      ProcessNewKeyFrame(); 
      
      // Check recent MapPoints
      MapPointCulling(); 

      // Triangulate New MapPoints
      CreateNewMapPoints(); 

      if(!CheckNewKeyFrames())
      {
        // Find more matches in neighbor keyframes and fuse point duplications
        SearchInNeighbors(); 
      }
      
      mbAbortBA = false; 
    
      if(!CheckNewKeyFrames() && !stopRequested())
      {
        // Local BA 
        if(mpMap->KeyFramesInMap() > 2)
        {
          // if(mbUsePlane)
          //  COptimizer::LocalBundleAdjustmentWithPlane(mpCurrentKeyFrame, &mbAbortBA, mpMap); 
          // else
            Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap); 
        }
        
        // Check redundant local Keyframes
        KeyFrameCulling(); 
      }
      
      mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
    }else if(Stop())
    {
      // Safe area to stop 
      while(isStopped() && !CheckFinish())
      {
        usleep(3000);
      }
      if(CheckFinish())
        break; 
    }

    ResetIfRequested(); 

    // Tracking will see that Local Mapping is busy 
    SetAcceptKeyFrames(true); 

    if(CheckFinish())
      break; 

    usleep(3000); 
  }
  SetFinish(); 
}

void CLocalMap::KeyFrameCulling()
{
  // Check redundant keyframes (only local keyframes)
  // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen in at least other 3 keyframes (in the same or finer scale)
  // we only consider close stereo points

  vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames(); 
  
  for(vector<KeyFrame*>::iterator vit = vpLocalKeyFrames.begin(), vend = vpLocalKeyFrames.end(); vit!=vend; ++vit)
  {
    KeyFrame* pKF = *vit; 
    if(pKF->mnId == 0)
      continue; 
    const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches(); 
    
    int nObs = 3; 
    const int thObs = nObs; 
    int nRedundantObservations = 0; 
    int nMPs = 0; 
    for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
    {
      MapPoint* pMP = vpMapPoints[i]; 
      if(pMP)
      {
        if(!pMP->isBad())
        {
          if(!mbMonocular)
          {
            if(pKF->mvDepth[i] > pKF->mThDepth || pKF->mvDepth[i] < 0)
              continue; 
          }
          nMPs++; 
          if(pMP->Observations() > thObs)
          {
            const int &scaleLevel = pKF->mvKeysUn[i].octave; 
            const map<KeyFrame*, size_t> observations = pMP->GetObservations(); 
            int nObs=0; 
            for(map<KeyFrame*, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; ++mit)
            {
              KeyFrame* pKFi = mit->first; 
              if(pKFi == pKF ) 
                continue;
              const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave; 

              if(scaleLeveli <= scaleLevel +1)
              {
                nObs++;
                if(nObs >= thObs)
                  break; 
              }
            }
            if(nObs >= thObs)
            {
              nRedundantObservations++;
            }
          }
        }
      }
    }
    if(nRedundantObservations > 0.9*nMPs)
      pKF->SetBadFlag();
  }
}

