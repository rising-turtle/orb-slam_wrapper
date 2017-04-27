/*
 *  Apr. 26 2017, He Zhang, hxzhang1@ualr.edu 
 *
 * test feature extraction process
 *
 * */

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"

using namespace ORB_SLAM2; 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_feature"); 
  ros::start(); 
  ros::NodeHandle nh; 

  string ORBvocfile("Vocabulary/ORBvoc.txt");
  string camParam("R200.yaml"); 
  string frgb("/home/etas/hxzhang/data/R200/obs1/color/1493223703.391451865.png");
  string fdpt("/home/etas/hxzhang/data/R200/obs1/depth/1493223703.391451865.exr"); 
  
  if(argc >= 3)
  {
    frgb = string(argv[1]); 
    fdpt = string(argv[2]); 
  }
  
  // Extractor 
  ORBextractor* ORBext = new ORBextractor(2000, 1.2, 8, 20, 7); 

  // ORB Vocabulary 
  ORBVocabulary * ORBvoc = new ORBVocabulary(); 
  ORBvoc->loadFromTextFile(ORBvocfile); 

  // calibration matrix 
  cv::Mat mk = cv::Mat::eye(3,3,CV_32F); 
  mk.at<float>(0,0) = 621.94176; // fx
  mk.at<float>(1,1) = 625.3561;  // fy
  mk.at<float>(0,2) = 319.77554; // cx
  mk.at<float>(1,2) = 236.21507; // cy

  // distortion matrix 
  cv::Mat mD(4,1,CV_32F); 
  mD.at<float>(0) = 0.08011; // k1
  mD.at<float>(1) = -0.56891;  // k2
  mD.at<float>(2) = 0.0000569487; // p1
  mD.at<float>(3) = -0.000209195; // p2

  float mbf = 47.; 
  float mThDepth = 100.; 

  // load img 
  cv::Mat imRGB, imD; 
  // Read image and depthmap from file
  imRGB = cv::imread(frgb,CV_LOAD_IMAGE_UNCHANGED);
  imD = cv::imread(fdpt,CV_LOAD_IMAGE_UNCHANGED);
  
  cv::Mat imGray = imRGB; 

  if(imRGB.channels() == 3)
    cvtColor(imGray, imGray, CV_BGR2GRAY); 

  Frame frame = Frame(imGray, imD, 0, ORBext, ORBvoc, mk, mD, mbf, mThDepth); 

  cout <<"frame extract "<<frame.N<<" ORB features!"<<endl; 
  // display the features 
  cv::Mat im_d(imGray.size(), CV_8UC3);
  cvtColor(imGray, im_d, CV_GRAY2RGB);
  for(int i=0; i<frame.N; i++)
  { 
    cv::KeyPoint& kp = frame.mvKeys[i]; 
    cv::circle(im_d, kp.pt, 4*(kp.octave+1), cv::Scalar(0,255,0), 1); 
  }

  cv::imshow("Original", im_d);
  cv::waitKey(0); 

  return 0; 
}
