/*
 *  Apr. 6 2017, He Zhang, hxzhang1@ualr.edu
 *
 * simple function to generate pc from imgs 
 *
 * */

#pragma once 

#include "cam_model.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

extern void generateColorPC(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >&, cv::Mat rgb, cv::Mat dpt, CamModel& cam); 


