#include "drake/multibody/fem/mpm-dev/KinematicCollisionObjects.h"

namespace drake {
namespace multibody {
namespace mpm {

void KinematicCollisionObjects::AddCollisionObject(
                    std::unique_ptr<AnalyticLevelSet> level_set,
                    math::RigidTransform<double> pose,
                    std::unique_ptr<SpatialVelocityTimeDependent>
                                                      spatial_velocity,
                    double friction_coeff) {
    CollisionObject::CollisionObjectState state = {std::move(pose),
                                                   std::move(spatial_velocity)};
    collision_objects_.emplace_back(
                        std::make_unique<CollisionObject>(std::move(level_set),
                                                          std::move(state),
                                                          friction_coeff));
}

int KinematicCollisionObjects::get_num_collision_objects() const {
    return collision_objects_.size();
}

void KinematicCollisionObjects::AdvanceOneTimeStep(double dt) {
    for (auto& obj : collision_objects_) {
        obj->AdvanceOneTimeStep(dt);
    }
}

void KinematicCollisionObjects::ApplyBoundaryConditions(
                                            const Vector3<double>& position,
                                            Vector3<double>* velocity) const {
    for (const auto& obj : collision_objects_) {
        obj->ApplyBoundaryCondition(position, velocity);
    }
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
