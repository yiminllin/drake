#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Partio.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/filesystem.h"
#include "drake/common/temp_directory.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/fem/mpm-dev/CorotatedElasticModel.h"
#include "drake/multibody/fem/mpm-dev/MPMDriver.h"
#include "drake/multibody/fem/mpm-dev/SpatialVelocityTimeDependent.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace multibody {
namespace mpm {

multibody::SpatialVelocity<double> left_hand_velocity(double t) {
    multibody::SpatialVelocity<double> wall_velocity;
    wall_velocity.rotational() = Vector3<double>(0.0, 0.0, 0.0);
    double freq = 0.4; // multiple of 0.04
    if (t < 0.36) {
        wall_velocity.translational() = Vector3<double>(0.5, 0.0, 0.0);
    } else if (t < 1.36) {
        wall_velocity.translational() = Vector3<double>(0.0, 0.0, 1.0);
    } else {
        if (static_cast<int>(trunc((t-1.36) / freq)) % 2 == 0) {
            wall_velocity.translational() = Vector3<double>(0.0, 0.0, -1.0);
        } else {
            wall_velocity.translational() = Vector3<double>(0.0, 0.0, 1.0);
        }
    }
    return wall_velocity;
}

multibody::SpatialVelocity<double> right_hand_velocity(double t) {
    multibody::SpatialVelocity<double> wall_velocity;
    wall_velocity.rotational() = Vector3<double>(0.0, 0.0, 0.0);
    double freq = 0.4;
    if (t < 0.36) {
        wall_velocity.translational() = Vector3<double>(-0.5, 0.0, 0.0);
    } else if (t < 1.36) {
        wall_velocity.translational() = Vector3<double>(0.0, 0.0, 1.0);
    } else {
        if (static_cast<int>(trunc((t-1.36) / freq)) % 2 == 0) {
            wall_velocity.translational() = Vector3<double>(0.0, 0.0, -1.0);
        } else {
            wall_velocity.translational() = Vector3<double>(0.0, 0.0, 1.0);
        }
    }
    return wall_velocity;
}

int DoMain() {
    MPMParameters::PhysicalParameters p_param {
        {0.0, 0.0, -9.81}                      // Gravitational acceleration
    };

    MPMParameters::SolverParameters s_param {
        // 3e-2,                                  // End time
        4e-0,                                  // End time
        1e-4,                                  // Time step size
        // 0.02,                                   // Grid size
        0.02,                                   // Grid size
    };

    MPMParameters::IOParameters io_param {
        "mpm-test",                            // case name
        "/home/yiminlin/Desktop/output",       // output directory name
        0.04,                                  // Interval of outputting
    };

    MPMParameters param {p_param, s_param, io_param};
    auto driver = std::make_unique<MPMDriver>(std::move(param));

    KinematicCollisionObjects objects = KinematicCollisionObjects();

    // Initialize the bottom wall
    SpatialVelocity<double> zero_velocity;
    zero_velocity.SetZero();
    std::unique_ptr<SpatialVelocityTimeDependent> bottom_wall_velocity_ptr =
        std::make_unique<SpatialVelocityTimeDependent>(zero_velocity);
    double bottom_wall_mu = 1.0;
    Vector3<double> bottom_wall_normal = {0.0, 0.0, 1.0};
    std::unique_ptr<AnalyticLevelSet> bottom_wall_level_set =
                        std::make_unique<HalfSpaceLevelSet>(bottom_wall_normal);
    Vector3<double> bottom_wall_translation = {0.0, 0.0, 0.0};
    math::RigidTransform<double> bottom_wall_pose =
                        math::RigidTransform<double>(bottom_wall_translation);
    objects.AddCollisionObject(std::move(bottom_wall_level_set),
                               std::move(bottom_wall_pose),
                               std::move(bottom_wall_velocity_ptr),
                               bottom_wall_mu);
    // Initialize the left wall
    std::unique_ptr<SpatialVelocityTimeDependent> left_hand_velocity_ptr =
        std::make_unique<SpatialVelocityTimeDependent>(left_hand_velocity);
    double left_hand_mu = 0.01;
    Vector3<double> left_hand_xscale = {0.1, 0.15, 0.15};
    std::unique_ptr<AnalyticLevelSet> left_hand_level_set =
                            std::make_unique<BoxLevelSet>(left_hand_xscale);
    Vector3<double> left_hand_translation = {-0.3, 0.0, 0.15};
    math::RigidTransform<double> left_hand_pose =
                            math::RigidTransform<double>(left_hand_translation);
    objects.AddCollisionObject(std::move(left_hand_level_set), std::move(left_hand_pose),
                               std::move(left_hand_velocity_ptr), left_hand_mu);

    // Initialize the right wall
    std::unique_ptr<SpatialVelocityTimeDependent> right_hand_velocity_ptr =
        std::make_unique<SpatialVelocityTimeDependent>(right_hand_velocity);
    double right_hand_mu = 0.01;
    Vector3<double> right_hand_xscale = {0.1, 0.15, 0.15};
    std::unique_ptr<AnalyticLevelSet> right_hand_level_set =
                            std::make_unique<BoxLevelSet>(right_hand_xscale);
    Vector3<double> right_hand_translation = {0.3, 0.0, 0.15};
    math::RigidTransform<double> right_hand_pose =
                            math::RigidTransform<double>(right_hand_translation);
    objects.AddCollisionObject(std::move(right_hand_level_set), std::move(right_hand_pose),
                               std::move(right_hand_velocity_ptr), right_hand_mu);

    // Initialize a sphere
    double radius = 0.05;
    SphereLevelSet level_set_sphere = SphereLevelSet(radius);
    Vector3<double> translation_sphere = {0.0, 0.0, 0.6};
    math::RigidTransform<double> pose_sphere =
                            math::RigidTransform<double>(translation_sphere);
    multibody::SpatialVelocity<double> velocity_sphere;
    velocity_sphere.translational() = Vector3<double>::Zero();
    velocity_sphere.rotational() = Vector3<double>{0.0, 0.0, 0.0};

    double E = 8e5;
    double nu = 0.49;
    std::unique_ptr<CorotatedElasticModel> elastoplastic_model
            = std::make_unique<CorotatedElasticModel>(E, nu);
    MPMDriver::MaterialParameters m_param_sphere{
                                                std::move(elastoplastic_model),
                                                1200,
                                                velocity_sphere,
                                                1
                                                };

    driver->InitializeKinematicCollisionObjects(std::move(objects));
    driver->InitializeParticles(level_set_sphere, pose_sphere,
                                std::move(m_param_sphere));
    driver->DoTimeStepping();

    return 0;
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake

int main() {
    return drake::multibody::mpm::DoMain();
}
