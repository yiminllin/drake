#include "drake/multibody/fem/mpm-dev/GravitationalForce.h"

namespace drake {
namespace multibody {
namespace mpm {

GravitationalForce::GravitationalForce():
                                gravitational_acceleration_(0.0, 0.0, -9.81) {}

GravitationalForce::GravitationalForce(Vector3<double> g):
                                gravitational_acceleration_(std::move(g)) {}

void GravitationalForce::ApplyGravitationalForces(double dt, Grid* grid) const {
    int i, j, k;
    // Gravitational acceleration
    for (const auto& [batch_index_flat, batch_index_3d] : grid->get_indices()) {
        i = batch_index_3d(0);
        j = batch_index_3d(1);
        k = batch_index_3d(2);
        const Vector3<double>& velocity_i = grid->get_velocity(i, j, k);
        grid->set_velocity(i, j, k,
                           velocity_i + dt*gravitational_acceleration_);
    }
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake

