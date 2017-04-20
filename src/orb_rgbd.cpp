#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

// #include"../../../include/System.h"
#include "system.h"
#include "System.h"
#include "cam_model.h"

using namespace std;


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    ros::NodeHandle nh;

    string ORBvoc("Vocabulary/ORBvoc.txt");
    string camParam("R200.yaml"); 
    // string imgDir("/media/work/work/data/realsense/03_26_ETAS_5F/03_26_19_19"); 
    // string imgDir("/home/davidz/work/data/real_sense/04_04_2017_xy"); 
    string imgDir("/home/davidz/work/data/real_sense/04_04_2017_emud"); 
    
    if(argc >= 2) 
      imgDir = string(argv[1]); 

    string no; 
    if(argc >=3)
      no = string(argv[2]);

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = imgDir + "/timestamp.txt";

    // string strAssociationFilename = string("rgbd_1410.txt");
    // string strAssociationFilename = string("/media/work/work/data/realsense/03_26_ETAS_5F/03_26_19_19");
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // cam model for f200
    CamModel cam(621.94176, 625.35610, 319.77554, 236.21507, 0.08011, -0.56891);
    cam.width = 640; 
    cam.height = 480; 

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM2::System SLAM(ORBvoc,camParam,ORB_SLAM2::System::RGBD,true);
    
    CSystem SLAM(ORBvoc,camParam,ORB_SLAM2::System::RGBD,true);
    
    // if(no != "no")
      // SLAM.enablePlanes(imgDir, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps, cam);  // set variables needed for plane feature

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    ros::spinOnce();

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(imgDir+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(imgDir+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << imgDir << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        // cout <<std::fixed<<" tframe = "<<tframe<<" imRGB = "<<vstrImageFilenamesRGB[ni]<<endl;
        // printf("tframe = %0.9lf imRGB = %s\n", tframe, vstrImageFilenamesRGB[ni].c_str());

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);
        usleep(10 * 1e3); // sleep 10 ms 
    }


    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.saveMapPoints("MapPoints.txt");
    
    ros::shutdown();

    return 0; 
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
    vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
  ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  while(!fAssociation.eof())
  {
    string s;
    getline(fAssociation,s);
    if(!s.empty())
    {
      stringstream ss;
      ss << s;
      double t;
      string sRGB, sD;
      ss >> t;
      // printf("load Images line: %s\n", ss.str().c_str());
      // printf("loadImages t = %.9lf\n", t);
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
      ss >> t;
      ss >> sD;
      vstrImageFilenamesD.push_back(sD);

    }
  }
}
