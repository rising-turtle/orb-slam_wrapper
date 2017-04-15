/*
 *  Apr.6 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  An interface for KeyFrame, by adding plane feature 
 *
 * */

#pragma once 

#include "KeyFrame.h"
#include "plane.h"

class ORB_SLAM2::Frame; 
class ORB_SLAM2::KeyFrameDatabase; 
class ORB_SLAM2::Map; 

class CKeyFrame : public ORB_SLAM2::KeyFrame
{
  public:
    CKeyFrame(ORB_SLAM2::Frame &F, ORB_SLAM2::Map * pMap, ORB_SLAM2::KeyFrameDatabase* pKFDB); 
    ~CKeyFrame(); 
    
    void erasePlaneObs(CPlaneF*); 
    void erasePlaneObs(size_t idx); 
    std::vector<CPlaneF*> getPlaneFeatures(); 
    CPlaneF* getPlaneFeatureAt(int idx); 
    CPlaneF* getPlaneObservationAt(int idx); 
    std::vector<CPlaneF*> mvpPlaneF; // plane features in the map
    std::vector<CPlaneF*> mvpCurrPF; // plane features in current frame
}; 
