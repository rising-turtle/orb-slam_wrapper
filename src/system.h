/*
 * Apr. 3, 2017 David Z , hxzhang1@ualr.edu
 *
 *  interface of ORB-SLAM's system, 
 *
 * */

// #ifndef SYSTEM_ZH_H
// #define SYSTEM_ZH_H
#pragma once 

#include "System.h"

class CTrack; 
class CamModel;
class CLocalMap; 
// using namespace ORB_SLAM2;

class CSystem : public ORB_SLAM2::System 
{
  public: 
    CSystem(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);
    ~CSystem(); 

    void enablePlanes(string imgDir, vector<string>& vRGBf, vector<string>& vDPTf, vector<double>& Timestamp, CamModel& cam); 
    void saveMapPoints(string filename); 

    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp); 
    CTrack* mpTrack;
    CLocalMap* mpLocalMap; 
    
    void setMask(cv::Mat mask); 
    cv::Mat getMask(); 
    bool bUseMask();
    bool isNewKF();
    float mpos[7];   // x, y, z, qx, qy, qz, qw
    void convertPos(cv::Mat); 
}; 


// #endif 
