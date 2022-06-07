#pragma once

#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/posed_half_space.h"

namespace drake {
namespace multibody {
namespace mpm {

// A Boundary condition class holding the boundary conditions' information in
// the domain. The boundaries in the MPM solver are represented as
// PosedHalfSpace. Points with nonpositive distances to the boundary half
// planes are inside the wall. We store the friction coefficients of each
// boundaries.
//            | normal direction             (outside of halfspace, dist > 0)
// ================================================================ wall
//                                           (inside of halfspace, dist < 0)
class BoundaryCondition {
 public:
    struct Boundary {
        double friction_coefficient;
        geometry::internal::PosedHalfSpace<double> boundary_space;
    };

    BoundaryCondition() = default;
    explicit BoundaryCondition(std::vector<Boundary> boundaries);

    int get_num_boundaries() const;
    const std::vector<Boundary>& get_boundaries() const;
    const Boundary& get_boundary(int index) const;

    void AddBoundary(Boundary boundary);

    // Applies the frictional wall boundary condition to the grid point with the
    // given position, where we apply the Coulumb friction law. Given the
    // velocity v, and denotes its normal and tangential components by vₙ =
    // (v ⋅ n)n, vₜ = v - vₙ, the impulse of colliding with the wall is given
    // by j = -m vₙ. The Coulumb friction law states the amount of friction
    // imposed is at most μ ‖j‖, where \mu is the friction coefficient of
    // the wall.
    // If ‖vₜ‖ <= μ‖vₙ‖,   v_new = 0.0
    // Otherwise    ,   v_new = vₜ - μ‖vₙ‖t, t - tangential direction
    // Then we overwrite the passed in velocity with v_new
    // For a grid point locates on multiple boundaries, we impose each boundary
    // condition, ordered as in boundaries_, to this grid point.
    // TODO(yiminlin.tri): May cause unexpected behavior at sharp corners
    void Apply(const Vector3<double>& position,
                     Vector3<double>* velocity) const;

 private:
    std::vector<Boundary> boundaries_;
};  // class BoundaryCondition

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
