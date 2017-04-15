/*
 * Apr.7 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  add plane vertex and plane edge in g2o, following the version in orb third-part
 *
 * */


#pragma once 

#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
// #include "types_six_dof_expmap.h"
#include "plane3d.h"
#include <Eigen/Core>
#include <iostream>

using namespace std;

namespace g2o{

class VertexPlane : public BaseVertex<3, Plane3D>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      VertexPlane(); 
    virtual bool read(std::istream& is); 
    virtual bool write(std::ostream& os) const;
    virtual void setToOriginImpl()
    {
      _estimate = Plane3D(); 
    }
    virtual void oplusImpl(const double* update)
    {
      Eigen::Map<const Vector3d> v(update); 
      _estimate.oplus(v); 
    }

    virtual bool setEstimateDataImpl(const double* est){
      Eigen::Map<const Vector4d> _est(est);
      _estimate.fromVector(_est);
      return true;
    }

    virtual bool getEstimateData(double* est) const{
      Eigen::Map<Vector4d> _est(est);
      _est = _estimate.toVector();
      return true;
    }

    virtual int estimateDimension() const {
      return 4;
    }
};

class EdgeSE3PlaneOnlyPose : public BaseUnaryEdge<3, Plane3D, VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3PlaneOnlyPose(); 
    virtual ~EdgeSE3PlaneOnlyPose();
   virtual bool read(std::istream& is); 
   virtual bool write(std::ostream& os) const; 

    void computeError()
    {
      const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]); 
      
      Plane3D z_estimated = v1->estimate()*mPw; 
      _error = z_estimated.ominus(_measurement); 
      // cout <<"in types_plabe_se3d.h: _error = "<<endl<<_error<<endl;
    }
    
    void setMeasurement(const Plane3D& m)
    {
      _measurement = m; 
    }

    // TODO: implement this function 
    // virtual void linearizeOplus();

    Plane3D mPw;  // plane3d expressed in world coordinate system
};

class EdgeSE3Plane : public BaseBinaryEdge<3, Plane3D, VertexPlane, VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE3Plane(); 
    virtual bool read(std::istream& is); 
    virtual bool write(std::ostream& os) const; 
  
    void computeError()
    {
      const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]); 
      const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[0]); 
      Plane3D z_estimated = v1->estimate() * v2->estimate(); 
      _error = z_estimated.ominus(_measurement); 
    }

    void setMeasurement(const Plane3D& m)
    {
      _measurement = m; 
    }
};


}



