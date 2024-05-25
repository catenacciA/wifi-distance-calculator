#include "../../include/optimize/DistanceFunctor.h"

double DistanceFunctor::operator()(const gtsam::Point3 &p1,
                                   const gtsam::Point3 &p2,
                                   gtsam::OptionalJacobian<1, 3> H1,
                                   gtsam::OptionalJacobian<1, 3> H2) const {
  if (H1) {
    *H1 = (p1 - p2).transpose().normalized();
  }
  if (H2) {
    *H2 = -(p1 - p2).transpose().normalized();
  }
  return (p1 - p2).norm();
}
