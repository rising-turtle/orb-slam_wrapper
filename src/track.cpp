#include "track.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "keyframe.h"
#include "optimizer.h"

using namespace ORB_SLAM2; 

CTrack::CTrack(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
                     KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor): 
  Tracking(pSys, pVoc, pFrameDrawer, pMapDrawer, pMap, pKFDB, strSettingPath, sensor), 
  mpExtractP(0),
  mbUsePlane(false)
{

}

CTrack::CTrack(const Tracking& pTrack) : Tracking(pTrack), 
  mpExtractP(0),
  mbUsePlane(false)
{

}

CTrack::~CTrack()
{
  if(mpExtractP != 0)
  {
    delete mpExtractP; 
    mpExtractP = 0; 
  }
}

void CTrack::enablePlanes(string imgDir, vector<string>& vRGBf, vector<string>& vDPTf, vector<double>& Timestamp, CamModel& cam)
{
  if(mpExtractP != NULL)
  {
    cout <<"track.cpp: mpExtractP has already been initialized, now reset it!"<<endl; 
    delete mpExtractP; 
  }
  mpExtractP = new CExtractPlane(cam); 

  mpExtractP->setData(imgDir, Timestamp, vRGBf, vDPTf); 
  mbUsePlane = true; 
  return ; 
}

cv::Mat CTrack::GrabImageRGBD(const cv::Mat& imRGB, const cv::Mat &imD, const double &timestamp)
{
  // return Tracking::GrabImageRGBD(imRGB, imD, timestamp); 
  mImGray = imRGB;
  cv::Mat imDepth = imD;

  if(mImGray.channels()==3)
  {
    if(mbRGB)
      cvtColor(mImGray,mImGray,CV_RGB2GRAY);
    else
      cvtColor(mImGray,mImGray,CV_BGR2GRAY);
  }
  else if(mImGray.channels()==4)
  {
    if(mbRGB)
      cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
    else
      cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
  }

  if(mDepthMapFactor!=1 || imDepth.type()!=CV_32F);
  imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

  mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

  Track();

  return mCurrentFrame.mTcw.clone();
}

int CTrack::numValidPoints() // check how many valid features have been observed in current frame 
{
  int ret = 0; 
  for(int i=0; i<mCurrentFrame.N; i++)
  {
    float z = mCurrentFrame.mvDepth[i];
    if(z>0)
    {
      ret++;
    }
  }
  return ret; 
}

bool CTrack::TrackWithMotionModel()
{
  ORBmatcher matcher(0.9, true); 

  // Update last frame pose according to its reference keyframe 
  // Create "visual odometry" points
  UpdateLastFrame(); 

  mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw); 

  fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL)); 

  int th; 

  if(mSensor!=System::STEREO)
    th = 15; 
  else
    th = 7; 

  int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor==System::MONOCULAR); 

  // If few matches, uses a wider window search 
  if(nmatches < 20)
  {
    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL)); 
    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2*th, mSensor == System::MONOCULAR); 
  }

  if(nmatches < 10)
  {
    cout <<"track.cpp: nmatches = "<<nmatches<<"failed tracking!"<<endl;
    return false; 
  }
  
  // Optimize frame pose with all matches 
  Optimizer::PoseOptimization(&mCurrentFrame); 

  // Discard outliers
  int nmatchesMap = 0; 
  for(int i=0; i<mCurrentFrame.N; i++)
  {
    if(mCurrentFrame.mvpMapPoints[i])
    {
      if(mCurrentFrame.mvbOutlier[i])
      {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i]; 
        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL); 
        mCurrentFrame.mvbOutlier[i] = false; 
        pMP->mbTrackInView = false; 
        pMP->mnLastFrameSeen = mCurrentFrame.mnId; 
        nmatches--;
      }
      else if(mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        ++nmatchesMap;
    }
  }

  if(mbOnlyTracking)
  {
    mbVO = nmatchesMap < 10; 
    return nmatches > 20; 
  }

  if(nmatchesMap < 10)
  {
    cout <<"track.cpp: TrackWithMotionModel() Failed: nmatchesMap = "<<nmatchesMap<<endl; 
  }
  return nmatchesMap >= 10;
}

bool CTrack::TrackReferenceKeyFrame()
{
  // Compute Bag of Words vector 
  mCurrentFrame.ComputeBoW(); 

  // We perform first an ORB matching with the reference keyframe
  // If enough matches are found we setup a PnP solver 
  ORBmatcher matcher(0.7, true);
  vector<MapPoint*> vpMapPointMatches; 

  int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches); 

  if(nmatches < 7)
  {
    cout <<"track.cpp() TrackReferenceKeyFrame() Failed: nmatches = "<<nmatches<<endl; 
    return false; 
  }
  
  mCurrentFrame.mvpMapPoints = vpMapPointMatches; 
  mCurrentFrame.SetPose(mLastFrame.mTcw); 

  Optimizer::PoseOptimization(&mCurrentFrame); 
  
  // Discard outliers
  int nmatchesMap = 0; 
  for(int i=0; i<mCurrentFrame.N; i++)
  {
    if(mCurrentFrame.mvpMapPoints[i])
    {
      if(mCurrentFrame.mvbOutlier[i])
      {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i]; 
        
        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL); 
        mCurrentFrame.mvbOutlier[i] = false;
        pMP->mbTrackInView = false; 
        pMP->mnLastFrameSeen = mCurrentFrame.mnId; 
        nmatches--;
      }else if(mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        ++nmatchesMap ; 
    }
  }

  if(nmatchesMap < 10)
  {
    cout << "track.cpp: TrackReferenceKeyFrame() Failed: nmatchesMap = "<<nmatchesMap<<endl; 
  }
  return nmatchesMap >=10;
}

void CTrack::Track()
{
 
  // do not handle case: only Tracking 
  if(mbOnlyTracking == true) 
    return Tracking::Track(); 

  // do not handle initialization stuff 
  if(mState==NO_IMAGES_YET)
  {
    mState = NOT_INITIALIZED; 
  }
  
  mLastProcessedState = mState; 
  mbNewKF = false; 

  // Get Map Mutex -> Map cannot be changed
  unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

  if(mState == NOT_INITIALIZED)
  {
      if(mSensor == System::STEREO || mSensor == System::RGBD)
        StereoInitialization(); 
      else
        MonocularInitialization(); 

      mpFrameDrawer->Update(this); 
      if(mState != OK)
        return; 
      
      // set current pose at identity
      cv::Mat pos = cv::Mat::eye(4,4, CV_32F); 
      mCurrentFrame.SetPose(pos); 

      // a new KF has been added 
      mbNewKF = true; 
  }
  else
  {
    // Track Frame.
    bool bOK;
    bool bRelocalFailed = false; 

    if(mState == OK)
    {
      // Local Mapping may have changed the position of some map points 
      CheckReplacedInLastFrame();

      if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
      {
        bOK = TrackReferenceKeyFrame();
      }
      else
      {
        bOK = TrackWithMotionModel();
        if(!bOK)
        {
          cout <<"TrackWithMotionModel failed try to TrackReferenceKeyFrame()"<<endl;
          bOK = TrackReferenceKeyFrame();
          if(!bOK)
          {
            cout <<"TrackReferenceKeyFrame failed as well!"<<endl;
          }
        }
      }
    }else // LOST 
    {
      bOK = Relocalization();
      if(bOK)
      {
        cout <<"Track recovers, Relocalization succeed ! "<<endl;
      }else
      {
        int nFPs = numValidPoints(); 
        if(nFPs > 50) // check the number of features in the current frame 
        {
          cout <<"track.cpp: current frame has feature points : "<<nFPs<<endl;
          bRelocalFailed = true; 
        }
        cout <<"Track lost, Relocalization failed ! current frame has valid points = "<<nFPs<<endl;
      }
    }

    mCurrentFrame.mpReferenceKF = mpReferenceKF;

    if(bOK)
    {
      bOK = TrackLocalMap();
      if(!bOK)
      {
        cout<<"TrackLocalMap failed! change state to TrackLost!"<<endl;
      }
    }

    if(bOK)
      mState = OK;
    else
    {
      cout <<"track.cpp: failed tracking, just ignore this frame!"<<endl; 
      // return ; 
      mState=LOST; // if failed 
    }

    // Update drawer
    mpFrameDrawer->Update(this);

    if(bRelocalFailed && !bOK)
    {
      cout <<"set current frame as tracked id = "<<mCurrentFrame.mnId<<endl;
      mCurrentFrame.SetPose(mLastFrame.mTcw); // assign it to the pose of last frame 
      bOK = true; 
      mState = OK; 
    }

    // If tracking were good, check if a new KF is needed
    if(bOK)
    {
      // update motion model 
      if(!mLastFrame.mTcw.empty())
      {
        cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
        mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
        mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
        mVelocity = mCurrentFrame.mTcw*LastTwc;
      }
      else
        mVelocity = cv::Mat();

      mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

      // clean temporal points matches
      for(int i=0; i<mCurrentFrame.N; i++)
      {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if(pMP)
          if(pMP->Observations()<1)
          {
            mCurrentFrame.mvbOutlier[i] = false;
            mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
          }
      }

      // delete temporal Mappoints 
      for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
      {
        MapPoint* pMP = *lit;
        delete pMP;
      }
      mlpTemporalPoints.clear();

      // check if a new KF is needed 
      if(NeedNewKeyFrame() || bRelocalFailed) // a Failed Frame, add it as a new keyframe 
      {
        CreateNewKeyFrame();
        mbNewKF = true; 
      }

      // delete outliers in this frame 
      for(int i=0; i<mCurrentFrame.N;i++)
      {
        if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
          mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
      }

      if(!mCurrentFrame.mpReferenceKF)
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

      mLastFrame = Frame(mCurrentFrame);
    }
  }

  // store frame pose information to retrieve complete trajectory information  
  if(!mCurrentFrame.mTcw.empty())
  {
    cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
    mlRelativeFramePoses.push_back(Tcr);
    mlpReferences.push_back(mpReferenceKF);
    mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
    mlbLost.push_back(mState==LOST);
  }
  else
  {
    // This can happen if tracking is lost
    mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
    mlpReferences.push_back(mlpReferences.back());
    mlFrameTimes.push_back(mlFrameTimes.back());
    mlbLost.push_back(mState==LOST);
  }

  // cout <<"mlRelativeFramePoses has pose: "<<mlRelativeFramePoses.size()<<endl;
  return ;
}

bool CTrack::isNewKF()
{
  return mbNewKF; 
}

bool CTrack::TrackLocalMap()
{
  // We have an estimation of the camera pose and some map points tracked in the frame.
  // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();
    
    SearchLocalPoints(); 

    if(mbUsePlane)
    {
      mbExtPlane = false; 
      if(mCurrentFrame.N < 20) // 20 when the number of inliers is small
      {
        findPlaneMatches(); 
       // new optimization function     
      COptimizer::PoseOptimizationWithPlane(&mCurrentFrame, mvpLocalPlanes, mvpCurrPlanes, mvPlaneAssociate);
        // Optimizer::PoseOptimization(&mCurrentFrame); // do not use plane 
        mbExtPlane = true; 
      }else
        Optimizer::PoseOptimization(&mCurrentFrame); // do not use plane 
    }else
    {
      // Optimize Pose
      Optimizer::PoseOptimization(&mCurrentFrame);
    }


    mnMatchesInliers = 0;

    // update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
      if(mCurrentFrame.mvpMapPoints[i])
      {
        if(!mCurrentFrame.mvbOutlier[i])
        {
          mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
          if(!mbOnlyTracking)
          {
            if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
              mnMatchesInliers++;
          }
          else
            mnMatchesInliers++;
        }
        else if(mSensor==System::STEREO)
          mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
      }
    }

    // Decide if a track is succeed 
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<30) // 50
    {
      cout <<"Failed : mnMatchesInliers = "<<mnMatchesInliers<<" currentId = "<<mCurrentFrame.mnId<<" mnLastRelocFrameId = "<<mnLastKeyFrameId<<" maxFrames = "<<mMaxFrames<<endl;
      return false;
    }

    if(mnMatchesInliers<20) // 30, 20 
    {
      cout <<"Failed : mnMatchesInliers: "<<mnMatchesInliers<<endl;
      return false;
    }
    else
      return true;
}

int CTrack::findPlaneMatches()
{
  // 1. collect local planes 
  mvpLocalPlanes.clear(); 
  for(vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; ++itKF)
  {
    KeyFrame* pKF1 = *itKF; 
    
    if(!pKF1) continue; 

    if(pKF1->mnId == 0) // initialized KF do not touch 
    { 
      continue; 
    }
    CKeyFrame* pKF = static_cast<CKeyFrame*>(pKF1); 
    const vector<CPlaneF*> vpPLs = pKF->getPlaneFeatures(); 
    for(vector<CPlaneF*>::const_iterator itPL=vpPLs.begin(), itEndPL=vpPLs.end(); itPL!=itEndPL; ++itPL)
    {
      CPlaneF* pPL = *itPL; 
      if(pPL == 0)
      {
        // cerr <<"track.cpp: what? pPL = 0"<< endl; // can be erased in the localMap thread 
        continue; 
      }
      if(pPL->mnTrackReferenceForFrame == mCurrentFrame.mnId)  // already added
        continue; 
      // cout <<"track.cpp: add plane id: "<<pPL->mId<<" Track ID: "<<pPL->mnTrackReferenceForFrame<<endl; 
      mvpLocalPlanes.push_back(pPL); 
      pPL->mnTrackReferenceForFrame = mCurrentFrame.mnId; 
    }
  }

  // 2. detect planes in the current frame 
  int num = 0;
  mvpCurrPlanes = mpExtractP->extractPlanes(mCurrentFrame.mTimeStamp);  
  mvPlaneAssociate.clear(); 
  vector<float> diff(mvpCurrPlanes.size(), -1); 
  map<int, vector<int> > asso; 
  double COS5 = cos(5.*M_PI/180.); 
  if(mvpCurrPlanes.size() > 0) // plane association 
  {
    mvPlaneAssociate.resize(mvpCurrPlanes.size(), -1); 
     // check with the local planes 
     for(int i=0; i<mvpCurrPlanes.size(); i++)
     {
        CPlaneF* pI = mvpCurrPlanes[i];     // plane in the current camera frame 
        // cv::Mat mwP = cv::Mat::zeros(4,1,CV_32F); 
        Eigen::Vector4d wPi; 
        pI->transform(mCurrentFrame.mTcw, wPi);  // actually we need mTwc, but Frame do not have that 
        Eigen::Vector4d wPj;
        for(int j=0; j<mvpLocalPlanes.size(); j++)
        {
          CPlaneF *pJ = mvpLocalPlanes[j];  // plane in the world frame
          pJ->getParam(wPj); 

          // compare pi and pj
          double COSA = wPi.head<3>().transpose()*wPj.head<3>(); 
          if(COSA < 0)
          {
            wPi = -1 * wPi; 
            COSA = -1 * COSA; 
          }
          double dis = fabs(wPi(3) - wPj(3));
          
          if(COSA < COS5) continue; 
          if(dis < 0.02) // find matched plane
          {
          cout << "track.cpp: find match between i = "<<i<<" j = "<<j<<" COSA = "<<COSA<<" angle: "<<acos(COSA)*180./M_PI<<endl<<" dis = "<<dis<<endl;
            mvPlaneAssociate[i] = j; 
            diff[i] = 1 - COSA + dis; 
            asso[j].push_back(i); 
            ++num; 
            // pI->setParam()
            break; 
          }
        }
     }
  }

  // 3. remove duplicate obs
  for(map<int, vector<int> >::iterator mit = asso.begin(), mend = asso.end(); mit != mend; ++mit)
  {
    // cout <<"for j = "<<mit->first<<" has "<<mit->second.size()<<" candidates!"<<endl;
    if(mit->second.size() >=2 )
    {
      float dJ = diff[mit->second[0]]; 
      vector<int>& vI = mit->second;  
      int pre_i = vI[0]; 

      for(int i=1; i<vI.size(); i++)
      {
        float dI = diff[vI[i]]; 
        if(dI < dJ) // current is better 
        { 
          // delete previous obs 
          mvPlaneAssociate[pre_i] = -1; 
          delete mvpCurrPlanes[pre_i]; 
          mvpCurrPlanes[pre_i] = NULL; 
          // cout <<"given j = "<<mit->first<<" keep "<<vI[i]<<" remove "<<pre_i<<endl;
          pre_i = vI[i];
          --num; 
        }else  // previous is better 
        {
          // delete current obs 
          int cur_i = vI[i]; 
          mvPlaneAssociate[cur_i] = -1; 
          delete mvpCurrPlanes[cur_i]; 
          mvpCurrPlanes[cur_i] = NULL; 
          // cout <<"given j = "<<mit->first<<" keep "<<pre_i<<" remove "<<vI[i]<<endl;
          --num; 
        }
      }
    } 
  }

  return num; 
}

void CTrack::CreateNewKeyFrame()
{
  if(mSensor == System::MONOCULAR) 
    return Tracking::CreateNewKeyFrame(); 

  if(!mpLocalMapper->SetNotStop(true))
    return;

  KeyFrame* pKF = new CKeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

  mpReferenceKF = pKF;
  mCurrentFrame.mpReferenceKF = pKF;
  
  mCurrentFrame.UpdatePoseMatrices(); 

  // We sort points by the measured depth by the stereo/RGBD sensor.
  // We create all those MapPoints whose depth < mThDepth.
  // If there are less than 100 close points we create the 100 closest.
 
  vector<pair<float, int> > vDepthIdx; 
  vDepthIdx.reserve(mCurrentFrame.N); 
  
  for(int i=0; i<mCurrentFrame.N; i++)
  {
    float z = mCurrentFrame.mvDepth[i]; 
    if(z > 0)
      vDepthIdx.push_back(make_pair(z,i)); 
  }
  
  if(!vDepthIdx.empty())
  {
    sort(vDepthIdx.begin(), vDepthIdx.end()); 
    int nPoints = 0; 
    for(size_t j=0; j<vDepthIdx.size(); j++)
    {
      int i = vDepthIdx[j].second; 
      bool bCreateNew = false; 
      MapPoint* pMP = mCurrentFrame.mvpMapPoints[i]; 
      if(!pMP)
        bCreateNew = true; 
      else if(pMP->Observations() < 1)
      {
        bCreateNew = true; 
        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL); 
      }

      if(bCreateNew)
      {
        cv::Mat x3D = mCurrentFrame.UnprojectStereo(i); 
        MapPoint* pNewMP = new MapPoint(x3D, pKF, mpMap); 
        pNewMP->AddObservation(pKF, i); 
        pKF->AddMapPoint(pNewMP, i); 
        pNewMP->ComputeDistinctiveDescriptors(); 
        pNewMP->UpdateNormalAndDepth(); 
        mpMap->AddMapPoint(pNewMP); 
        
        mCurrentFrame.mvpMapPoints[i] = pNewMP; 
        nPoints++;
      }else{
        nPoints++;
      }

      if(vDepthIdx[j].first > mThDepth && nPoints > 100)
        break; 
    }
  }
  
  // add plane features as well 
  if(mbUsePlane)
  {
    if(mbExtPlane == false)
      findPlaneMatches();  // process for current frame

    int index = 0; 
    for(size_t i=0; i<mvPlaneAssociate.size(); i++)
    {
      CPlaneF* pI = mvpCurrPlanes[i]; 
      if(pI == NULL) continue; // this plane has been deleted
      int j = mvPlaneAssociate[i]; 
      if(j == -1) // a new plane
      {
         CPlaneF* pW = pI->transformToWorld(pKF); 
         ((CKeyFrame*)pKF)->mvpPlaneF.push_back(pW); 
         // index =  ((CKeyFrame*)pKF)->mvpPlaneF.size() - 1; 
         pW->addObservation(pKF, i); 
      }else // an old plane, add observation 
      {
         CPlaneF* pW = mvpLocalPlanes[j]; 
        ((CKeyFrame*)pKF)->mvpPlaneF.push_back(pW); 
         pW->addObservation(pKF, i); 
         
         // check whether need to flip plane observation 
         Eigen::Vector4d wPi;  Eigen::Vector4d wPj;
         pI->transform(pKF->GetPose(), wPi);  // actually we need mTwc, but Frame do not have that 
         pW->getParam(wPj); 
         double COSA = wPi.head<3>().transpose()*wPj.head<3>(); 
         if(COSA < 0) // need to flip pI
         {
           pI->flipParam(); 
         }
         pW->mnObs ++; 
      }  
    }
    // add the extracted planes in the current frame into KF, for later local bundle adjustment 
    ((CKeyFrame*)pKF)->mvpCurrPF.swap(mvpCurrPlanes); 
  }

  mpLocalMapper->InsertKeyFrame(pKF); 
  mpLocalMapper->SetNotStop(false); 
  mnLastKeyFrameId = mCurrentFrame.mnId; 
  mpLastKeyFrame = pKF; 
  return ; 
}
