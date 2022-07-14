#include "drake/multibody/fem/mpm-dev/GravitationalForce.h"

namespace drake {
namespace multibody {
namespace mpm {

GravitationalForce::GravitationalForce():
                                gravitational_acceleration_(0.0, 0.0, -9.81) {}

GravitationalForce::GravitationalForce(Vector3<double> g):
                                gravitational_acceleration_(std::move(g)) {}

void GravitationalForce::ApplyGravitationalForces(double dt, Grid* grid) const {
    // Gravitational acceleration
    for (const auto& [batch_index_flat, batch_index_3d] : grid->get_indices()) {
        // Skip grid points with zero mass
        if (grid->get_mass(batch_index_flat) != 0.0) {
            const Vector3<double>& velocity_i
                                        = grid->get_velocity(batch_index_flat);
            grid->set_velocity(batch_index_flat,
                            velocity_i + dt*gravitational_acceleration_);
        }
    }
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake

