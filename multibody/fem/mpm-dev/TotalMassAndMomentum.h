#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {

// Store the sum of mass, momentum and angular momentum of the grid/particles
// The angular momentum is about the origin in the world frame
struct TotalMassAndMomentum {
    double sum_mass;
    Vector3<double> sum_momentum;
    Vector3<double> sum_angular_momentum;
};

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
