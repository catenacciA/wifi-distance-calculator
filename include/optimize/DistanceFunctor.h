#ifndef DISTANCEFUNCTOR_H
#define DISTANCEFUNCTOR_H

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Expression.h>

class DistanceFunctor {
public:
  double operator()(const gtsam::Point3 &p1, const gtsam::Point3 &p2,
                    gtsam::OptionalJacobian<1, 3> H1 = boost::none,
                    gtsam::OptionalJacobian<1, 3> H2 = boost::none) const;
};

#endif // DISTANCEFUNCTOR_H
