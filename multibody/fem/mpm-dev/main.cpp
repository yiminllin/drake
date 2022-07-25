#include <array>
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
#include "drake/multibody/fem/mpm-dev/StvkHenckyWithVonMisesModel.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace multibody {
namespace mpm {

int DoMain() {
    MPMParameters::PhysicalParameters p_param {
        {0.0, 0.0, -9.81}                      // Gravitational acceleration
    };

    MPMParameters::SolverParameters s_param {
        2e-0,                                  // End time
        // 5e-4,                                  // Time step size
        // 0.025,                                   // Grid size
        4e-4,                                  // Time step size
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

    // Initialize the wall
    multibody::SpatialVelocity<double> wall_velocity;
    wall_velocity.SetZero();
    double wall_mu = 0.5;
    Vector3<double> wall_normal = {0.0, 0.0, 1.0};
    std::unique_ptr<AnalyticLevelSet> wall_level_set =
                            std::make_unique<HalfSpaceLevelSet>(wall_normal);
    Vector3<double> wall_translation = {0.0, 0.0, 0.0};
    math::RigidTransform<double> wall_pose =
                            math::RigidTransform<double>(wall_translation);
    objects.AddCollisionObject(std::move(wall_level_set), std::move(wall_pose),
                               wall_velocity, wall_mu);

    // Initialize the cylinder
    multibody::SpatialVelocity<double> cylinder_velocity;
    cylinder_velocity.translational() = Vector3<double>(-1.0, 0.0, 0.0);
    // Choose consistent rotational velocity so
    // rotational velocity x (0, 0, -r) = - translational velocity
    cylinder_velocity.rotational() = Vector3<double>(0.0, -2.5, 0.0);
    double cylinder_mu = 0.4;
    double cylinder_height = 2;
    double cylinder_radius = 0.025;
    std::unique_ptr<AnalyticLevelSet> cylinder_level_set =
                            std::make_unique<CylinderLevelSet>(cylinder_height,
                                                               cylinder_radius);
    math::RollPitchYaw cylinder_rpw = {M_PI/2.0, 0.0, 0.0};
    Vector3<double> cylinder_translation = {0.8, 0.1, 0.06};
    math::RigidTransform<double> cylinder_pose =
            math::RigidTransform<double>(cylinder_rpw, cylinder_translation);
    objects.AddCollisionObject(std::move(cylinder_level_set),
                               std::move(cylinder_pose),
                               cylinder_velocity, cylinder_mu);

    // Initialize a sphere
    double radius = 0.04;
    SphereLevelSet level_set_sphere = SphereLevelSet(radius);
    Vector3<double> translation_sphere = {0.4, 0.1, 0.05};
    math::RigidTransform<double> pose_sphere =
                            math::RigidTransform<double>(translation_sphere);
    multibody::SpatialVelocity<double> velocity_sphere;
    velocity_sphere.translational() = Vector3<double>::Zero();
    velocity_sphere.rotational() = Vector3<double>{0.0, 0.0, 0.0};

    double E = 8e4;
    double nu = 0.49;
    double yield_stress = 0.01*E;
    std::unique_ptr<StvkHenckyWithVonMisesModel> elastoplastic_model
            = std::make_unique<StvkHenckyWithVonMisesModel>(E, nu,
                                                            yield_stress);
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
