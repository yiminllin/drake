#include "drake/multibody/fem/mpm-dev/SparseGrid.h"

namespace drake {
namespace multibody {
namespace mpm {

SparseGrid::SparseGrid(double h): num_active_gridpts_(0), h_(h),
                                  active_gridpts_(0) {}

void SparseGrid::reserve(double capacity) {
    index_map_.reserve(capacity);
    active_gridpts_.reserve(capacity);
    velocities_.reserve(capacity);
    masses_.reserve(capacity);
    forces_.reserve(capacity);
}

int SparseGrid::get_num_active_gridpt() const {
    return num_active_gridpts_;
}

double SparseGrid::get_h() const {
    return h_;
}

Vector3<double> SparseGrid::get_position(const Vector3<int>& index_3d) const {
    return Vector3<double>(index_3d(0)*h_, index_3d(1)*h_, index_3d(2)*h_);
}

const Vector3<double>& SparseGrid::get_velocity(const Vector3<int>& index_3d)
                                                                        const {
    return velocities_[Reduce3DIndex(index_3d)];
}

double SparseGrid::get_mass(const Vector3<int>& index_3d) const {
    return masses_[Reduce3DIndex(index_3d)];
}

const Vector3<double>& SparseGrid::get_force(const Vector3<int>& index_3d)
                                                                        const {
    return forces_[Reduce3DIndex(index_3d)];
}

const Vector3<double>& SparseGrid::get_velocity(size_t index_1d) const {
    return velocities_[index_1d];
}

void SparseGrid::set_velocity(const Vector3<int>& index_3d,
                        const Vector3<double>& velocity) {
    velocities_[Reduce3DIndex(index_3d)] = velocity;
}

void SparseGrid::set_mass(const Vector3<int>& index_3d, double mass) {
    masses_[Reduce3DIndex(index_3d)] = mass;
}

void SparseGrid::set_force(const Vector3<int>& index_3d,
                     const Vector3<double>& force) {
    forces_[Reduce3DIndex(index_3d)] = force;
}

void SparseGrid::set_velocity(size_t index_1d,
                              const Vector3<double>& velocity) {
    velocities_[index_1d] = velocity;
}

void SparseGrid::AccumulateVelocity(const Vector3<int>& index_3d,
                                    const Vector3<double>& velocity) {
    velocities_[Reduce3DIndex(index_3d)] += velocity;
}

void SparseGrid::AccumulateMass(const Vector3<int>& index_3d, double mass) {
    masses_[Reduce3DIndex(index_3d)] += mass;
}

void SparseGrid::AccumulateForce(const Vector3<int>& index_3d,
                                 const Vector3<double>& force) {
    forces_[Reduce3DIndex(index_3d)] += force;
}

void SparseGrid::UpdateActiveGridPoints(
                                    std::vector<Vector3<int>> active_gridpts) {
    num_active_gridpts_ = active_gridpts.size();
    active_gridpts_ = std::move(active_gridpts);

    for (int i = 0; i < num_active_gridpts_; ++i) {
        index_map_[active_gridpts_[i]] = i;
    }
}

void SparseGrid::RescaleVelocities() {
    for (int i = 0; i < num_active_gridpts_; ++i) {
        velocities_[i] = velocities_[i] / masses_[i];
    }
}

void SparseGrid::ResetStates() {
    std::fill(masses_.begin(), masses_.begin()+num_active_gridpts_, 0.0);
    std::fill(velocities_.begin(), velocities_.begin()+num_active_gridpts_,
              Vector3<double>::Zero());
    std::fill(forces_.begin(), forces_.begin()+num_active_gridpts_,
              Vector3<double>::Zero());
}

size_t SparseGrid::Reduce3DIndex(const Vector3<int>& index_3d) const {
    return index_map_.at(index_3d);
}

Vector3<int> SparseGrid::Expand1DIndex(size_t index_1D) const {
    return active_gridpts_[index_1D];
}

void SparseGrid::UpdateVelocity(double dt) {
    for (int i = 0; i < num_active_gridpts_; ++i) {
        velocities_[i] += dt*forces_[i]/masses_[i];
    }
}

void SparseGrid::EnforceBoundaryCondition(
                                    const KinematicCollisionObjects& objects) {
    // For all grid points, enforce frictional wall boundary condition
    for (int i = 0; i < num_active_gridpts_; ++i) {
        objects.ApplyBoundaryConditions(get_position(active_gridpts_[i]),
                                        &velocities_[i]);
    }
}

TotalMassAndMomentum SparseGrid::GetTotalMassAndMomentum() const {
    TotalMassAndMomentum sum_state;
    sum_state.sum_mass             = 0.0;
    sum_state.sum_momentum         = Vector3<double>::Zero();
    sum_state.sum_angular_momentum = Vector3<double>::Zero();
    for (int i = 0; i < num_active_gridpts_; ++i) {
        double mi = get_mass(active_gridpts_[i]);
        const Vector3<double> vi = get_velocity(active_gridpts_[i]);
        const Vector3<double> xi = get_position(active_gridpts_[i]);
        sum_state.sum_mass             += mi;
        sum_state.sum_momentum         += mi*vi;
        sum_state.sum_angular_momentum += mi*xi.cross(vi);
    }
    return sum_state;
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
