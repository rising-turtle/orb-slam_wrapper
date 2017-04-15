/*
 *  Apr. 7 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  simple test for types_plane_3d 
 *
 * */

#include <iostream>
#include "types_plane_se3d.h"
#include <Eigen/Core>
#include "Converter.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

using namespace std; 
using namespace g2o;
using namespace ORB_SLAM2;

void test_pose_optimize(); 
void test_bundle_plane(); 

int main(int argc, char* argv[])
{
  // test_pose_optimize(); 
  test_bundle_plane(); 
   
  return 0; 
}

void test_bundle_plane()
{
  int pid = 10; 
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
  g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);
 
  Eigen::Matrix<double ,4 ,4> T = Eigen::Matrix<double, 4, 4>::Identity(); 
  // T(0,3) = 2.; T(1, 3) = -1; T(2,3) = 0.5; 
  cv::Mat cvT = Converter::toCvMat(T); 
  
  // add a vertex 
  g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
  vSE3->setEstimate(Converter::toSE3Quat(cvT));
  vSE3->setId(0);
  vSE3->setFixed(true);
  optimizer.addVertex(vSE3);

  // add another vertex  
  T(0,3) = 1.; cvT = Converter::toCvMat(T);  
  vSE3 = new g2o::VertexSE3Expmap();
  vSE3->setEstimate(Converter::toSE3Quat(cvT));
  vSE3->setId(1);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);

  // add a plane node 
  Eigen::Vector4d m; m << -1, 0, 0, -2.5; 
  VertexPlane * vPL = new VertexPlane(); 
  vPL->setEstimate(Plane3D(m)); 
  vPL->setId(pid); 
  vPL->setMarginalized(true); 
  optimizer.addVertex(vPL); 

  // add a plane edge
  g2o::EdgeSE3Plane* e= new g2o::EdgeSE3Plane(); 
  e->setVertex(1, dynamic_cast<OptimizableGraph::Vertex*>(optimizer.vertex(0))); 
  e->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer.vertex(pid)));
  
  m << -1, 0, 0, -3; 
  e->setMeasurement(Plane3D(m)); 
  Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*1; 
  e->setInformation(Info);
  optimizer.addEdge(e); 

  // add another plane edge 
  m << -1, 0, 0, -1; 
  g2o::EdgeSE3Plane* e2 = new g2o::EdgeSE3Plane(); 
  e2->setMeasurement(Plane3D(m));
  e2->setVertex(1, dynamic_cast<OptimizableGraph::Vertex*>(optimizer.vertex(1))); 
  e2->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer.vertex(pid)));
  Info = Eigen::Matrix3d::Identity()*10; 
  e2->setInformation(Info); 
  optimizer.addEdge(e2); 

  optimizer.initializeOptimization();
  e->computeError(); 
  e2->computeError(); 
  double chi2 = e->chi2(); 

  cout<<"before edge e1 error = "<<chi2<<" e2 error = "<<e2->chi2()<<endl; 
  optimizer.optimize(2);

  e->computeError(); e2->computeError(); 
  chi2 = e->chi2(); 

  cout <<"after edge e1 error = "<<chi2<<" e2 error = "<<e2->chi2()<<endl;
  g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(1));
  g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  cv::Mat pose = Converter::toCvMat(SE3quat_recov);

  cout <<"output pose: "<<endl<<pose<<endl;

  g2o::VertexPlane * vPL_recv = static_cast<g2o::VertexPlane*>(optimizer.vertex(pid)); 
  Plane3D pl3d = vPL_recv->estimate(); 
  
  cout <<"output plane: "<<endl<<pl3d.coeffs()<<endl;

}


void test_pose_optimize()
{
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
  g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);
  

  Eigen::Matrix<double ,4 ,4> T = Eigen::Matrix<double, 4, 4>::Identity(); 
  T(0,3) = 2.; T(1, 3) = -1; T(2,3) = 0.5; 
  cv::Mat cvT = Converter::toCvMat(T); 
  
  // add a vertex 
  g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
  vSE3->setEstimate(Converter::toSE3Quat(cvT));
  vSE3->setId(0);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);

  // add a plane edge
  g2o::EdgeSE3PlaneOnlyPose* e= new g2o::EdgeSE3PlaneOnlyPose(); 
  e->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer.vertex(0))); 

  Eigen::Vector4d m; m << 0, 0, -1, 5; 
  e->setMeasurement(Plane3D(m)); 
  Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*10; 
  e->setInformation(Info);
  m << 0, 0, -1, 10;
  e->mPw = Plane3D(m);  
  optimizer.addEdge(e); 

  optimizer.initializeOptimization(0);
  e->computeError(); 
  double chi2 = e->chi2(); 

  cout<<"before edge error = "<<chi2<<endl; 
  optimizer.optimize(2);

  e->computeError(); 
  chi2 = e->chi2(); 

  cout <<"edge error = "<<chi2<<endl;
  g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
  g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  cv::Mat pose = Converter::toCvMat(SE3quat_recov);

  cout <<"output pose: "<<endl<<pose<<endl;

}
