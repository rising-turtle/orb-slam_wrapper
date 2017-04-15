/*
 * Apr. 9 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  an interface for LocalMapping 
 *
 * */

#pragma once 

#include "LocalMapping.h"

class CLocalMap : public ORB_SLAM2::LocalMapping
{
  public:
    CLocalMap(ORB_SLAM2::Map* pMap, const float bMonocular); 
    ~CLocalMap();

    // Main function 
    void Run(); 
    bool mbUsePlane; 

};
