/*
 *  Apr. 6, 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  specific for ORB-SLAM, extract pc given a frame's timestamp, and extract planes from this pc
 *
 * */

#pragma once

#include "plane.h"
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "cam_model.h"

class CExtractPlane 
{
  public:
    CExtractPlane(CamModel& ); 
    ~CExtractPlane(); 

    std::vector<CPlaneF*> extractPlanes(double timestamp, double squared_dis_threshold = 0.0004); 
    std::vector<CPlaneF*> extractPlanes(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >& pPC, double squared_dis_threshold = 0.0004); 
    bool extractPointCloud(double timestamp, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >& pPC); 

    CamModel mCam; 
    
    void setData(string Dir, vector<double>& vTimeStamps, vector<string>& vFRGB, vector<string>& vFDPT ); 

    // frame rgb and dpt
    int findCurrentFrameId(double timestamp); 
    string mDir; // where to store the dataset 
    vector<double> mvTimestamps; 
    vector<string> mvstrImageFilenamesRGB; 
    vector<string> mvstrImageFilenamesD; 

};
