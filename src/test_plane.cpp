/*
 * Apr.6, 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  test extracting planes from the data of RealSense F200 
 *
 * */

#include <ros/ros.h>
#include "detect_plane.h"
#include "glob_def.h"
#include "vtk_viewer.h"

string imgDir("/home/davidz/work/data/real_sense/04_04_2017_emud2"); 

// Retrieve paths to images
vector<string> vstrImageFilenamesRGB;
vector<string> vstrImageFilenamesD;
vector<double> vTimestamps;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
    vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void prepare(int argc, char* argv[]);

void test1();

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    ros::NodeHandle nh;

    prepare(argc, argv); 
    
    test1();

  return 0;
}

void test1()
{
    // 
    CamModel cam(621.94176, 625.35610, 319.77554, 236.21507, 0.08011, -0.56891);
    cam.width = 640; 
    cam.height = 480; 
    CExtractPlane ext(cam); 
    ext.setData(imgDir, vTimestamps, vstrImageFilenamesRGB, vstrImageFilenamesD); 

    ros::NodeHandle np("~"); 
    int from_id = 1; 
    int to_id = 1000;
    int id_step = 4; 
    np.param("from_id", from_id, from_id); 
    np.param("to_id", to_id, to_id); 
    np.param("id_step", id_step, id_step);

    for(int i=from_id; i<to_id && i < vTimestamps.size(); i+= id_step)
    {
      double timestamp = vTimestamps[i]; 
      CloudPtr pPC(new Cloud); 
      ROS_INFO("test_plane.cpp: try to handle frame at time %lf", timestamp);

      if(ext.extractPointCloud(timestamp, pPC))
      {
        ROS_INFO("test_plane.cpp: succeed to extract point cloud with %d points", pPC->size()); 

        vector<CPlaneF*> vPFs = ext.extractPlanes(pPC); 
        // ROS_INFO("test_plane.cpp: extract %d planes!", nPlane); 
        
        // label the first plane 
        if(vPFs.size() >= 1)
        {
          ROS_WARN("test_plane.cpp: extract %d planes!", vPFs.size()); 
          CPlaneF* pPF = vPFs[0]; 
          for(int j=0; j<pPC->points.size(); j++)
          {
            Point& p = pPC->points[j]; 
            double dis = pPF->dis2Plane(p.x, p.y, p.z); 
            if(fabs(dis) < 0.01)
            {
              p.r = 0; p.g = 255; p.b = 0;  // mark plane points as green 
            }
          }
        }else
        {
          // 
          ROS_INFO("test_plane.cpp: no plane has been detected!");
        }
        
        // show it 
        CVTKViewer<Point> viewer; 
        viewer.getViewer()->addCoordinateSystem(0.2,0, 0, 0); 
        viewer.addPointCloud(pPC, "showing_pcs");
        while(!viewer.stopped())
        {
          viewer.runOnce(); 
          usleep(100000);
        }
      }
    }
}

void prepare(int argc, char* argv[])
{
    // string ORBvoc("Vocabulary/ORBvoc.txt");
    // string camParam("F200.yaml"); 
    if(argc >= 2) 
      imgDir = string(argv[1]); 
    string strAssociationFilename = imgDir + "/timestamp.txt";

    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
    return ; 
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
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
      ss >> t;
      ss >> sD;
      vstrImageFilenamesD.push_back(sD);
    }
  }
  ROS_INFO("succeed to load %d records", vTimestamps.size()); 
}










