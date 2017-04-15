
#include "optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "types/types_plane_se3d.h"

#include "Converter.h"
#include "plane.h"
#include "keyframe.h"

using namespace ORB_SLAM2; 
using namespace std; 
using namespace g2o;

void COptimizer::LocalBundleAdjustmentWithPlane(KeyFrame* pKF, bool *pbStopFlag, ORB_SLAM2::Map *pMap)
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    
    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; ++i)
    {
      KeyFrame *pKFi = vNeighKFs[i]; 
      pKFi->mnBALocalForKF = pKF->mnId; 
      if(!pKFi->isBad())
        lLocalKeyFrames.push_back(pKFi); 
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints; 
    for(list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit !=lend; ++lit)
    {
      vector<MapPoint*> vpMaps = (*lit)->GetMapPointMatches(); 
      for(vector<MapPoint*>::iterator vit = vpMaps.begin(), vend=vpMaps.end(); vit != vend; ++vit)
      {
        MapPoint* pMP = *vit; 
        if(pMP)
        {
          if(!pMP->isBad())
          {
            if(pMP->mnBALocalForKF != pKF->mnId)
            {
              lLocalMapPoints.push_back(pMP); 
              pMP->mnBALocalForKF = pKF->mnId; 
            }
          }
        }
      }
    }

    // Planes seen in the Local KeyFrames 
    list<CPlaneF*> lLocalPlanes; 
    for(list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit!=lend; lit++)
    {
      KeyFrame* pKFi = *lit; 
      if(pKFi->mnId <= 0) continue; // ignore the first KF 
        
      CKeyFrame * pKFc = static_cast<CKeyFrame*>(pKFi); 
      vector<CPlaneF*> vPLs = pKFc->getPlaneFeatures(); 
      
      for(vector<CPlaneF*>::iterator vit = vPLs.begin(), vend = vPLs.end(); vit != vend; ++vit)
      {
          CPlaneF* pPF = *vit; 
          if(pPF)
          {
            if(!pPF->isBad())
            {
              if(pPF->mnBALocalForKF != pKF->mnId)
              {
                // cout <<"pPF->mId = "<<pPF->mId<<" pPF->mnBALocalForKF = "<<pPF->mnBALocalForKF<<endl;
                lLocalPlanes.push_back(pPF); 
                pPF->mnBALocalForKF = pKF->mnId; 
              }
            }
          }
      }
    }

    // Fixed KeyFrames. KeyFrames that see Local MapPoints but that are not Local KeyFrames 
    list<KeyFrame*> lFixedCameras; 
    for(list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; ++lit)
    {
      map<KeyFrame*, size_t> observations = (*lit)->GetObservations(); 
      for(map<KeyFrame*, size_t>::iterator mit=observations.begin(), mend = observations.end(); mit != mend; ++mit)
      {
        KeyFrame * pKFi = mit->first; 
        if(pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
        {
          pKFi->mnBAFixedForKF = pKF->mnId; 
          if(!pKFi->isBad())
            lFixedCameras.push_back(pKFi); 
        }
      }
    }

    // add Fixed KeyFrames
    for(list<CPlaneF*>::iterator lit = lLocalPlanes.begin(), lend = lLocalPlanes.end(); lit != lend; ++lit)
    {
      map<KeyFrame*, size_t> observations = (*lit)->getObservations(); 
      for(map<KeyFrame*, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; ++mit)
      {
        KeyFrame* pKFi = mit->first; 
        // cout <<"handle plane id = "<<(*lit)->mId<<" observed by keyframe id = "<<pKFi->mnId<<endl; 
        if(pKFi->isBad())
        {
          // cout <<"pKFi = "<<pKFi->mnId<<" is bad continue!"<<endl;
          continue; // this KeyFrame can be erased later 
        }
        if(pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
        {
          // cout <<"pKFi.id = "<<pKFi->mnId<<" added to FixedGroup since pKFi->mnBAFixedForKF = "<<pKFi->mnBAFixedForKF<<endl;
          pKFi->mnBAFixedForKF = pKF->mnId; 
          lFixedCameras.push_back(pKFi); 
        }else{
          // cout <<"pKFi.id = "<<pKFi->mnId<<" failed to FixedGroup since pKFi->mnBALocalForKF = "<<pKFi->mnBALocalForKF<<" pKFi->mnBAFixedForKF = "<<pKFi->mnBAFixedForKF<<" and pKF->mnId = "<<pKF->mnId<<endl;
        }
      }
    }

    // Setup optimizer 
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
      optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0; 
    unsigned long maxPointId = 0; 

    // Set Local KeyFrame vertices 
    for(list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; ++lit)
    {
      KeyFrame * pKFi = *lit; 
      g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap(); 
      vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose())); 
      vSE3->setId(pKFi->mnId); 
      vSE3->setFixed(pKFi->mnId == 0); // if this vertice is the first one 
      optimizer.addVertex(vSE3); 
      if(pKFi->mnId > maxKFid)
        maxKFid = pKFi->mnId; 
    }

    // Set Fixed KeyFrame vertices 
    for(list<KeyFrame*>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; ++lit)
    {
      KeyFrame* pKFi = *lit; 
      g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap(); 
      vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose())); 
      vSE3->setId(pKFi->mnId); 
      vSE3->setFixed(true); 
      optimizer.addVertex(vSE3); 
      if(pKFi->mnId > maxKFid)
        maxKFid = pKFi->mnId; 
    }
    
    // Set MapPoint vertices 
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size())*lLocalMapPoints.size(); 

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono; 
    vpEdgesMono.reserve(nExpectedSize); 

    vector<KeyFrame*> vpEdgeKFMono; 
    vpEdgeKFMono.reserve(nExpectedSize); 

    vector<MapPoint*> vpMapPointEdgeMono; 
    vpMapPointEdgeMono.reserve(nExpectedSize); 

    vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo; 
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);
    const float thHuberPlane = sqrt(7.815); 

    // add point features  
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
      MapPoint *pMP = *lit; 
      g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ(); 
      vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos())); 
      int id = pMP->mnId + maxKFid + 1; 
      if(id > maxPointId) // find out the id of the first plane node 
        maxPointId = id; 
      vPoint->setId(id); 
      vPoint->setMarginalized(true);  //
      optimizer.addVertex(vPoint); 
      
      const map<KeyFrame*, size_t> observations = pMP->GetObservations(); 

      // Set Edges
      for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend = observations.end(); mit != mend; mit++)
      {
        KeyFrame * pKFi = (*mit).first; 
        if(!pKFi->isBad())
        {
          const cv::KeyPoint & kpUn = pKFi->mvKeysUn[mit->second]; 
          
          // Monocular observations 
          if(pKFi->mvuRight[mit->second] < 0)
          {
            Eigen::Matrix<double, 2,1> obs; 
            obs << kpUn.pt.x , kpUn.pt.y ; 
            g2o::EdgeSE3ProjectXYZ * e = new g2o::EdgeSE3ProjectXYZ(); 
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id))); // PointXYZ node 
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId))); // Pose node 
            e->setMeasurement(obs); 
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]; 
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2); 
            g2o::RobustKernelHuber * rk = new g2o::RobustKernelHuber; 
            e->setRobustKernel(rk); 
            rk->setDelta(thHuberMono); 
            e->fx = pKFi->fx; e->fy = pKFi->fy; e->cx = pKFi->cx; e->cy = pKFi->cy; 
            
            optimizer.addEdge(e);
            vpEdgesMono.push_back(e); 
            vpEdgeKFMono.push_back(pKFi); 
            vpMapPointEdgeMono.push_back(pMP); 
          }else // Stereo observations
          {
            Eigen::Matrix<double, 3,1> obs; 
            const float kp_ur = pKFi->mvuRight[mit->second]; 
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur; 
            
            g2o::EdgeStereoSE3ProjectXYZ * e = new g2o::EdgeStereoSE3ProjectXYZ(); 
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id))); // PointXYZ node 
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId))); // Pose node 
            e->setMeasurement(obs); 
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]; 
            e->setInformation(Eigen::Matrix3d::Identity() * invSigma2); 
            g2o::RobustKernelHuber * rk = new g2o::RobustKernelHuber; 
            e->setRobustKernel(rk); 
            rk->setDelta(thHuberStereo); 
            e->fx = pKFi->fx; e->fy = pKFi->fy; e->cx = pKFi->cx; e->cy = pKFi->cy; 
            e->bf = pKFi->mbf; 

            optimizer.addEdge(e); 
            vpEdgesStereo.push_back(e); 
            vpEdgeKFStereo.push_back(pKFi); 
            vpMapPointEdgeStereo.push_back(pMP); 
          }
        }
      }
    }

    // add plane features
    int nExpectedPlaneSize = lLocalKeyFrames.size()*lLocalPlanes.size(); 
    vector<g2o::EdgeSE3Plane*> vpEdgesPlane; 
    vpEdgesPlane.reserve(nExpectedPlaneSize); 
    
    vector<KeyFrame*> vpEdgeKFPlane; 
    vpEdgeKFPlane.reserve(nExpectedPlaneSize); 

    vector<CPlaneF*> vpPlaneEdge; 
    vpPlaneEdge.reserve(nExpectedPlaneSize); 

    Eigen::Vector4d m; 
    int weight = (lLocalMapPoints.size() / lLocalPlanes.size()) / 2 + 1; 

    // cout <<"optimizer.cpp: maxPointId: "<<maxPointId<<endl;

    for(list<CPlaneF*>::iterator lit = lLocalPlanes.begin(), lend = lLocalPlanes.end(); lit!=lend; ++lit)
    {
      CPlaneF* pPF = *lit; 
      pPF->getParam(m); 
      g2o::VertexPlane* vPL = new g2o::VertexPlane(); 
      vPL->setEstimate(Plane3D(m)); 
      int id = pPF->mId + maxPointId + 1; 
      // cout <<" pF id: "<<pPF->mId<<" vPL id = "<<id<<endl;
      vPL->setId(id); 
      vPL->setMarginalized(true); 
      optimizer.addVertex(vPL); 

      // add obervation edges 
      map<KeyFrame*, size_t> observations = pPF->getObservations(); 
      Eigen::Vector4d lm; 
      for(map<KeyFrame*, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; ++mit)
      {
        KeyFrame* pKFi = (*mit).first; 
        if(pKFi && !pKFi->isBad() && pKFi->mnId <= maxKFid)
        {
          CKeyFrame* pKFc = static_cast<CKeyFrame*>(pKFi); 
          CPlaneF* obs = pKFc->getPlaneObservationAt(mit->second); 
          if(obs != NULL && !obs->isBad())
          {
            // cout <<" add Edge from Pose: "<<pKFc->mnId<<" to plane: "<<pPF->mId<<endl;
            obs->getParam(lm); 
            // add edge 
            g2o::EdgeSE3Plane* e = new g2o::EdgeSE3Plane(); 
            g2o::OptimizableGraph::Vertex* pnode = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)); 
            if(pnode == NULL)
            {
              cerr <<"Plane node = NULL at Vertex id = "<<id<<" will crash!"<<endl; 
            }
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id))); // Plane node  
            pnode = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFc->mnId));
            if(pnode == NULL)
            {
              cerr <<"pose node = NULL at Vertex id = "<<pKFc->mnId<<" will crash!"<<endl; 
            }

            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFc->mnId))); // Pose node 
            e->setMeasurement(Plane3D(lm)); 
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * weight; 
            e->setInformation(Info); 
            g2o::RobustKernelHuber * rk = new g2o::RobustKernelHuber; 
            e->setRobustKernel(rk); 
            rk->setDelta(thHuberPlane); 
            // cout <<"before addEdge()"<<endl; 
            optimizer.addEdge(e); 
            // cout <<"after addEdge()"<<endl;

            vpEdgesPlane.push_back(e); 
            vpEdgeKFPlane.push_back(pKFi); 
            vpPlaneEdge.push_back(pPF); 
           }
        }
      }
    }

    if(pbStopFlag)
      if(*pbStopFlag)
        return;
    optimizer.initializeOptimization(); 
    optimizer.optimize(5);

    bool bDoMore = true; 

   if(pbStopFlag)
      if(*pbStopFlag)
        bDoMore = false; 
    
   if(bDoMore)
   {
    // cout <<"bDoMore yes!"<<endl;
      // check inlier observations 
      for(size_t i=0, iend = vpEdgesMono.size(); i<iend; i++)
      {
        g2o::EdgeSE3ProjectXYZ * e = vpEdgesMono[i]; 
        MapPoint *pMP = vpMapPointEdgeMono[i]; 

        if(pMP->isBad())
          continue; 
        
        if(e->chi2() > 5.991 || !e->isDepthPositive())
        {
          e->setLevel(1);
        }
        
        e->setRobustKernel(0);
      }

      for(size_t i=0, iend = vpEdgesStereo.size(); i < iend; i++)
      {
        g2o::EdgeStereoSE3ProjectXYZ * e = vpEdgesStereo[i]; 
        MapPoint * pMP = vpMapPointEdgeStereo[i]; 
          
        if(pMP->isBad())
          continue; 
        if(e->chi2() > 7.815 || !e->isDepthPositive())
        {
          e->setLevel(1); 
        }
        
        e->setRobustKernel(0); 
      }
    
      // check planes 
      for(size_t i=0, iend = vpEdgesPlane.size(); i<iend; ++i)
      {
        g2o::EdgeSE3Plane * e = vpEdgesPlane[i]; 
        CPlaneF* pPL = vpPlaneEdge[i]; 
        if(pPL->isBad()) continue; 
        
        if(e->chi2() > 7.815)
        {
          e->setLevel(1); 
        }

        e->setRobustKernel(0); 
      }

      // optimize again by removing outliers 
      
      optimizer.initializeOptimization(0); 
      optimizer.optimize(10);

     // cout <<"finish DoMore"<<endl;
   }
    
   // cout <<"start inlier check!"<<endl; 
   vector<pair<KeyFrame*, MapPoint*> > vToErase; 
   vToErase.reserve(vpEdgesMono.size()+ vpEdgesStereo.size()); 

   // check inliers 
   for(size_t i=0, iend = vpEdgesMono.size(); i<iend; i++)
   {
      g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i]; 
      MapPoint* pMP = vpMapPointEdgeMono[i]; 
      if(pMP->isBad()) continue; 

      if(e->chi2() > 5.991 || !e->isDepthPositive())
      {
        KeyFrame* pKFi = vpEdgeKFMono[i]; 
        vToErase.push_back(make_pair(pKFi, pMP)); 
      }
   }

  for(size_t i=0, iend = vpEdgesStereo.size(); i<iend; i++)
  {
    g2o::EdgeStereoSE3ProjectXYZ * e= vpEdgesStereo[i]; 
    MapPoint* pMP = vpMapPointEdgeStereo[i]; 
    if(pMP->isBad()) continue; 

    if(e->chi2() > 7.815 || !e->isDepthPositive())
    {
      KeyFrame* pKFi = vpEdgeKFStereo[i]; 
      vToErase.push_back(make_pair(pKFi, pMP)); 
    }
  }
  
  // Get Map Mutex
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);
  
  if(!vToErase.empty())
  {
    for(size_t i=0; i<vToErase.size(); i++)
    {
      KeyFrame* pKFi = vToErase[i].first; 
      MapPoint* pMP = vToErase[i].second; 
      pKFi->EraseMapPointMatch(pMP); 
      pMP->EraseObservation(pKFi); 
    }
  }
  
  // check inlier planes 
  for(size_t i=0, iend = vpEdgesPlane.size(); i<iend; ++i)
  {
    g2o::EdgeSE3Plane * e = vpEdgesPlane[i]; 
    CPlaneF* pPL = vpPlaneEdge[i]; 
    if(pPL->isBad()) continue; 

    if(e->chi2() > 7.815)
    {
      // erase this observation 
      KeyFrame* pKFi = vpEdgeKFPlane[i]; 
      CKeyFrame* pKFc = static_cast<CKeyFrame*>(pKFi); 
      
      CPlaneF* pPL = vpPlaneEdge[i]; 
      // cout <<"erase plane node id : "<<pPL->mId + maxPointId + 1<<endl;
      pKFc->erasePlaneObs(pPL); 
      pPL->eraseObservation(pKFc); 
    }
  }
  // Recover optimized data 
 // cout <<"after inlier check ! start recover "<<endl; 

  // KeyFrames
  for(list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit != lend; ++lit)
  {
    KeyFrame* pKFi = *lit; 
    g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId)); 
    g2o::SE3Quat SE3quat = vSE3->estimate(); 
    pKFi->SetPose(Converter::toCvMat(SE3quat)); 
  }

  // Points
  for(list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend= lLocalMapPoints.end(); lit != lend; ++lit)
  {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + maxKFid + 1)); 
    pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate())); 
    pMP->UpdateNormalAndDepth(); 
  }

  // Planes 
  for(list<CPlaneF*>::iterator lit = lLocalPlanes.begin(), lend = lLocalPlanes.end(); lit != lend; ++lit)
  {
    CPlaneF* pPL = *lit; 
    g2o::VertexPlane* vPL = static_cast<g2o::VertexPlane*>(optimizer.vertex(pPL->mId + maxPointId + 1)); 
    Eigen::Vector4d pm = vPL->estimate().toVector();
    pPL->setParam(pm); 
  }
 // cout <<"finish recover !"<<endl;
  return ; 
}

int COptimizer::PoseOptimizationWithPlane(Frame* pFrame, vector<CPlaneF*>& vLocalP, vector<CPlaneF*>& vCurrP, vector<int>& vAssociate)
{
  g2o::SparseOptimizer optimizer; 
  g2o::BlockSolver_6_3::LinearSolverType * linearSolver; 
  
  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>(); 

  g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver); 

  g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
  optimizer.setAlgorithm(solver); 

  
  int nInitialCorrespondences = 0; 
  
  // set frame vertex
  g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap(); 
  vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw)); 
  vSE3->setId(0); 
  vSE3->setFixed(false); 
  optimizer.addVertex(vSE3); 
  
  // set MapPoint vertices
  const int N = pFrame->N; 

  vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
  vector<size_t> vnIndexEdgeMono;
  vpEdgesMono.reserve(N);
  vnIndexEdgeMono.reserve(N);

  vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo; 
  vector<size_t> vnIndexEdgeStereo; 
  vpEdgesStereo.reserve(N); 
  vnIndexEdgeStereo.reserve(N); 
 
  const int M = vAssociate.size(); 
  vector<bool> outlier(M, false); 
  vector<g2o::EdgeSE3PlaneOnlyPose*> vpEdgesPlane; 
  vector<size_t> vnIndexEdgePlane; 
  vpEdgesPlane.reserve(M); 
  vnIndexEdgePlane.reserve(M); 

  const float deltaMono = sqrt(5.991);
  const float deltaStereo = sqrt(7.815); 
  const float deltaPlane = sqrt(7.815); 

  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex); 

    for(int i=0; i<N; i++)
    {
      MapPoint* pMP = pFrame->mvpMapPoints[i]; 
      if(pMP)
      {
        // Monocular obervation  
        if(pFrame->mvuRight[i] < 0)
        {
          nInitialCorrespondences++; 
          pFrame->mvbOutlier[i] = false ; 
          
          Eigen::Matrix<double, 2,1> obs; 
          const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i]; 
          obs << kpUn.pt.x , kpUn.pt.y; 

          g2o::EdgeSE3ProjectXYZOnlyPose * e = new g2o::EdgeSE3ProjectXYZOnlyPose(); 
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
          e->setMeasurement(obs); 
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]; 
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2); 

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber; 
          e->setRobustKernel(rk); 
          rk->setDelta(deltaMono); 

          e->fx = pFrame->fx; e->fy = pFrame->fy; e->cx = pFrame->cx; e->cy = pFrame->cy; 
          cv::Mat Xw = pMP->GetWorldPos(); 
          e->Xw[0] = Xw.at<float>(0); 
          e->Xw[1] = Xw.at<float>(1);
          e->Xw[2] = Xw.at<float>(2); 

          optimizer.addEdge(e); 

          vpEdgesMono.push_back(e); 
          vnIndexEdgeMono.push_back(i); 
        }
        else // Stereo observation 
        {
          nInitialCorrespondences++; 
          pFrame->mvbOutlier[i] = false; 
          
          // set edge 
          Eigen::Matrix<double, 3, 1> obs; 
          const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i]; 
          const float &kp_ur = pFrame->mvuRight[i]; 
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur; 
          
          g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose(); 
          
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0))); 
          e->setMeasurement(obs); 
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]; 
          Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2; 
          e->setInformation(Info); 

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber; 
          e->setRobustKernel(rk); 
          rk->setDelta(deltaStereo); 

          e->fx = pFrame->fx; e->fy = pFrame->fy; e->cx = pFrame->cx; e->cy = pFrame->cy; 
          e->bf = pFrame->mbf; 
          cv::Mat Xw = pMP->GetWorldPos(); 
          
          e->Xw[0] = Xw.at<float>(0); 
          e->Xw[1] = Xw.at<float>(1);
          e->Xw[2] = Xw.at<float>(2); 
          
          optimizer.addEdge(e); 
          vpEdgesStereo.push_back(e); 
          vnIndexEdgeStereo.push_back(i);
        }
      }
    }
  }

  if(nInitialCorrespondences < 3)
    return 0; 

  int nPlane = 0; 
  for(size_t i=0; i< vAssociate.size(); i++)
  {
    if(vAssociate[i] >= 0) ++nPlane; 
  }

  if(nPlane > 0)
  {
    int nAlpha = (nInitialCorrespondences / nPlane)/2; 
    if(nAlpha < 1) nAlpha = 1;  

    // try to add plane observations 
    for(size_t i=0; i< vAssociate.size(); i++)
    {
      int j = vAssociate[i]; 
      if(j == -1) continue; 

      CPlaneF* wP = vLocalP[j]; 
      CPlaneF* cP = vCurrP[i]; 

      // construct plane edge 
      g2o::EdgeSE3PlaneOnlyPose* e = new g2o::EdgeSE3PlaneOnlyPose(); 
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0))); 

      Eigen::Vector4d m; 
      cP->getParam(m); 
      e->setMeasurement(Plane3D(m)); 
      wP->getParam(m); 
      e->mPw = Plane3D(m); 
      Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*nAlpha;
      e->setInformation(Info); 

      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber; 
      e->setRobustKernel(rk); 
      rk->setDelta(deltaPlane); 

      optimizer.addEdge(e); 
      vpEdgesPlane.push_back(e); 
      vnIndexEdgePlane.push_back(i); 
    }
  }
  
  // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
  const float chi2Mono[4]={5.991,5.991,5.991,5.991};
  const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
  const float chi2Plane[4]={7.815,7.815,7.815, 7.815};
  const int its[4]={10,10,10,10}; 
  
  int nBad = 0; 
  for(size_t it=0; it<4; it++)
  {
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]); 

    nBad = 0; 
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
    {
      g2o::EdgeSE3ProjectXYZOnlyPose * e= vpEdgesMono[i]; 
      const size_t idx = vnIndexEdgeMono[i]; 

      if(pFrame->mvbOutlier[idx])
      {
        e->computeError(); 
      }
      
      const float chi2 = e->chi2(); 
      if(chi2 > chi2Mono[it])
      {
        pFrame->mvbOutlier[idx] = true; 
        e->setLevel(1); 
        nBad++;
      }else
      {
        pFrame->mvbOutlier[idx] = false; 
        e->setLevel(0);
      }
      
      if(it == 2) // last iteration 
        e->setRobustKernel(0);
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++ )
    {
      g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i]; 
      const size_t idx = vnIndexEdgeStereo[i]; 

      if(pFrame->mvbOutlier[idx])
      {
        e->computeError(); 
      }
      double chi2 = e->chi2();
      if(chi2 > chi2Stereo[it])
      {
        pFrame->mvbOutlier[idx] = true; 
        e->setLevel(1); 
        nBad++; 
      }else
      {
        pFrame->mvbOutlier[idx] = false; 
        e->setLevel(0); 
      }
      if(it == 2)
        e->setRobustKernel(0); 
    }
    
    for(size_t i=0, iend=vpEdgesPlane.size(); i<iend; i++)
    {
      g2o::EdgeSE3PlaneOnlyPose * e = vpEdgesPlane[i]; 
      int idx = vnIndexEdgePlane[i]; 
      double chi2 = e->chi2(); 
      if(outlier[idx])
      {
        e->computeError(); 
      }
      if(chi2 > chi2Plane[it])
      {
        outlier[idx] = true;
        e->setLevel(1); 
      }else
      {
        outlier[idx] = false; 
        e->setLevel(0); 
      }
      if(it == 2)
        e->setRobustKernel(0); 
    }

    if(optimizer.edges().size() < 10)
      break; 
  }
 
  // handle false plane association 
  for(size_t i=0; i< vAssociate.size(); i++)
  {
    if(vAssociate[i] >= 0 && outlier[i] == true) 
      vAssociate[i] = -1; 
  }

  // Recover optimized pose and return number of inliers
  g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
  g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  cv::Mat pose = Converter::toCvMat(SE3quat_recov);
  pFrame->SetPose(pose);

  return nInitialCorrespondences - nBad; 
}
