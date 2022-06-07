#include "drake/multibody/fem/mpm-dev/BoundaryCondition.h"

namespace drake {
namespace multibody {
namespace mpm {

BoundaryCondition::BoundaryCondition(std::vector<Boundary> boundaries):
                                        boundaries_(std::move(boundaries)) {}

int BoundaryCondition::get_num_boundaries() const {
    return boundaries_.size();
}

const std::vector<BoundaryCondition::Boundary>&
                                    BoundaryCondition::get_boundaries() const {
    return boundaries_;
}

const BoundaryCondition::Boundary&
                            BoundaryCondition::get_boundary(int index) const {
    DRAKE_ASSERT(index < boundaries_.size());
    return boundaries_[index];
}

void BoundaryCondition::AddBoundary(BoundaryCondition::Boundary boundary) {
    boundaries_.emplace_back(std::move(boundary));
}

void BoundaryCondition::Apply(const Vector3<double>& position,
                                    Vector3<double>* velocity) const {
    // For all boundaries
    for (const auto& boundary : boundaries_) {
        // If the grid point is on/in the boundary, enforce the frictional wall
        // boundary condition
        if (boundary.boundary_space.CalcSignedDistance(position) <= 0) {
            // Normal vector of the boundary plane
            const Vector3<double>& n_i = boundary.boundary_space.normal();
            // Normal and tangential components of the velocity
            Vector3<double> vn = velocity->dot(n_i)*n_i;
            Vector3<double> vt = *velocity - vn;
            // Normal and tangential speed
            double vn_norm = vn.norm();
            double vt_norm = vt.norm();

            // Limit the amount of friction to at most eliminating the
            // tangential velocity
            if (vt_norm <= boundary.friction_coefficient*vn_norm) {
                *velocity = Vector3<double>::Zero();
            } else {
                // If the tangential velocity is zero, the updated velocity is
                // zero by above.
                *velocity = vt - boundary.friction_coefficient
                                *vn_norm*vt/vt_norm;
            }
        }
    }
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
