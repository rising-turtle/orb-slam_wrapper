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

