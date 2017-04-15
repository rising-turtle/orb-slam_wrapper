/*
 *  Apr. 7 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  optimize graph by adding plane features 
 *
 *
 * */

#pragma once

#include "Optimizer.h"
#include <vector>

using namespace std; 
class CPlaneF; 

class COptimizer : public ORB_SLAM2::Optimizer
{
  public:
    int static PoseOptimizationWithPlane(ORB_SLAM2::Frame* pFrame, vector<CPlaneF*>& vLocalP, vector<CPlaneF*>& vCurrP, vector<int>& vAssociate); 
    void static LocalBundleAdjustmentWithPlane(ORB_SLAM2::KeyFrame* pKF, bool *pbStopFlag, ORB_SLAM2::Map* pMap);
};


