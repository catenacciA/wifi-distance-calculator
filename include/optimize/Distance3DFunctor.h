#ifndef DISTANCE3D_FUNCTOR_H
#define DISTANCE3D_FUNCTOR_H

#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/slam/expressions.h>

class Distance3DFunctor {
public:
  gtsam::Vector3
  operator()(const gtsam::Point3 &p1, const gtsam::Point3 &p2,
             gtsam::OptionalJacobian<3, 3> H1 = boost::none,
             gtsam::OptionalJacobian<3, 3> H2 = boost::none) const {
    if (H1)
      *H1 = gtsam::Matrix3::Identity();
    if (H2)
      *H2 = -gtsam::Matrix3::Identity();
    return p1 - p2;
  }
};

#endif // DISTANCE3D_FUNCTOR_H
