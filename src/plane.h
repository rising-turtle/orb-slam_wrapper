/*
 * Apr. 6, 2017 He Zhang, hxzhang1@ualr.edu
 *
 * plane feature 
 *
 * */

#pragma once

#include "KeyFrame.h"
#include <Eigen/Core>

class CPlaneF
{
  public:
    CPlaneF(const cv::Mat& nv_d, ORB_SLAM2::KeyFrame* pRefKF); 
    CPlaneF(const cv::Mat& nv_d ) ; // , ORB_SLAM2::Frame* pFrame);
    ~CPlaneF(); 

    cv::Mat getParam(); // parameters in world frame 
    void getParam(Eigen::Vector4d& v);
    void setParam(Eigen::Vector4d& v); 
    void flipParam(); 
    float dis2Plane(float px, float py, float pz); 

    std::map<ORB_SLAM2::KeyFrame*, size_t> getObservations(); 
    void addObservation(ORB_SLAM2::KeyFrame* pKF, size_t idx); 
    void eraseObservation(ORB_SLAM2::KeyFrame* pKF); 
    int getIndexKF(ORB_SLAM2::KeyFrame*); 

    CPlaneF* transformToWorld(ORB_SLAM2::KeyFrame* pKF); 
    void transform(cv::Mat cvT, Eigen::Vector4d& nw);
    static long int sNext; 
    int mId; 
    int mnObs;       // times being observed 
    int mnTrackReferenceForFrame; 
    int mnBALocalForKF; 
    // ORB_SLAM2::Frame* mpFrame; 
    ORB_SLAM2::KeyFrame* mpKF; 
    cv::Mat mParam; // parameters of the plane in world coordinate system 
    std::map<ORB_SLAM2::KeyFrame*, size_t> mmObs;  // store observation history    
    bool mbBad; // identify whether this plane is valid 
    void setBadFlag(); 
    bool isBad(); 

    std::mutex mMutexFeatures; 
    std::mutex mMutexPos; 

};

