#include <iostream>
#include "types_plane_se3d.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace g2o{

  VertexPlane::VertexPlane() : 
    BaseVertex<3, Plane3D>(){}

  bool VertexPlane::read(std::istream& is) 
  {
    Vector4d lv;
    for (int i=0; i<4; i++)
      is >> lv[i];
    setEstimate(Plane3D(lv));
    // is >> color(0) >> color(1) >> color(2);
    return true;
  }

  bool VertexPlane::write(std::ostream& os) const {
    Vector4d lv=_estimate.toVector();
    for (int i=0; i<4; i++){
      os << lv[i] << " ";
    }
    // os << color(0) << " " << color(1) << " " << color(2) << " ";
    return os.good();
  }


  EdgeSE3PlaneOnlyPose::EdgeSE3PlaneOnlyPose() : 
    BaseUnaryEdge<3, Plane3D, VertexSE3Expmap>()
  {}

  EdgeSE3PlaneOnlyPose::~EdgeSE3PlaneOnlyPose(){}

  bool EdgeSE3PlaneOnlyPose::read(std::istream& is)
  {
    Eigen::Vector4d v; 
    is >> v(0) >> v(1) >> v(2) >> v(3); 
    setMeasurement(Plane3D(v));

    for(int i=0; i<information().rows(); ++i)
      for(int j=0; j<information().cols(); ++j)
      {
        is >> information()(i,j); 
        if(i != j)
          information()(j,i) = information()(i, j);
      }
    return true;
  }

  bool EdgeSE3PlaneOnlyPose::write(std::ostream& os) const
  {
    Eigen::Vector4d v = _measurement.toVector(); 
    os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
    // os << color(0) << " " << color(1) << " " << color(2) << " ";
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);
    return os.good();
  }

  EdgeSE3Plane::EdgeSE3Plane() : 
    BaseBinaryEdge<3, Plane3D, VertexPlane, VertexSE3Expmap>() {}
  
  bool EdgeSE3Plane::read(std::istream& is)
  {
    Eigen::Vector4d v; 
    is >> v(0) >> v(1) >> v(2) >> v(3); 
    setMeasurement(Plane3D(v));

    for(int i=0; i<information().rows(); ++i)
      for(int j=0; j<information().cols(); ++j)
      {
        is >> information()(i,j); 
        if(i != j)
          information()(j,i) = information()(i, j);
      }
    return true;
  }
  
  bool EdgeSE3Plane::write(std::ostream& os) const
  {
    Eigen::Vector4d v = _measurement.toVector(); 
    os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
    // os << color(0) << " " << color(1) << " " << color(2) << " ";
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);
    return os.good();
  }
}
