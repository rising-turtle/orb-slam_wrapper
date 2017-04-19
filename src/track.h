/*
 * Apr.3, 2017, David Z hxzhang1@ualr.edu
 *
 *  interface of the tracking 
 *
 * */

#pragma once

#include <vector>
#include "Tracking.h"
#include "detect_plane.h"

class CamModel; 
using namespace std; 

class CTrack : public ORB_SLAM2::Tracking
{
  public:
    CTrack( ORB_SLAM2::System* pSys, ORB_SLAM2::ORBVocabulary* pVoc, ORB_SLAM2::FrameDrawer* pFrameDrawer, 
        ORB_SLAM2::MapDrawer* pMapDrawer, ORB_SLAM2::Map* pMap, ORB_SLAM2::KeyFrameDatabase* pKFDB, 
        const string &strSettingPath, const int sensor); 
    CTrack(const ORB_SLAM2::Tracking& ); 
    ~CTrack();
    
    void Track();
    bool TrackWithMotionModel(); 
    bool TrackReferenceKeyFrame(); 
    bool TrackLocalMap(); 
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
  
    void enablePlanes(string imgDir, vector<string>& vRGBf, vector<string>& vDPTf, vector<double>& Timestamp, CamModel& cam);

    void CreateNewKeyFrame();

    int numValidPoints(); // check how many valid features have been observed in current frame 

    // plane related functions 
    int findPlaneMatches(); 
    bool mbUsePlane; // whether use plane feature 
    bool mbExtPlane; // current frame has been processed for extracting planes 
    vector<CPlaneF*> mvpCurrPlanes;   // planes in the current Frame
    vector<int> mvPlaneAssociate;     // planes association result
    vector<CPlaneF*> mvpLocalPlanes;  // planes in the Local Map
    CExtractPlane* mpExtractP;      // extract planes from frame
 }; 
