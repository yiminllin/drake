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
#include "drake/multibody/fem/mpm-dev/SpatialVelocityTimeDependent.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace multibody {
namespace mpm {

void velocity_field(Vector3<double>, double, Vector3<double>*) { }

int DoMain() {
    Vector3<double> translation_sphere2 = {10.0, 10.0, 10.0};

    // // // 4/SQRT(3) = 0.23094, two spheres very close,
    // Vector3<double> translation_sphere2 = {0.231, 0.231, 0.231};

    double v_strength = 50.0;
    MPMParameters::PhysicalParameters p_param {
        {0.0, 0.0, 0.0},                      // Gravitational acceleration
        velocity_field,
    };

    MPMParameters::SolverParameters s_param {
        0.2,                                  // End time
        5e-4,                                  // Time step size
        0.025,                                   // Grid size
        0.5,                                  // CFL
    };

    MPMParameters::IOParameters io_param {
        "mpm-test",                            // case name
        "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/colliding-sphere/outputs-fast-slowmo",       // output directory name
        0.001,                                  // Interval of outputting
    };

    MPMParameters param {p_param, s_param, io_param};
    auto driver = std::make_unique<MPMDriver>(std::move(param));

    KinematicCollisionObjects objects = KinematicCollisionObjects();

    // Initialize a sphere
    double radius = 0.2;
    SphereLevelSet level_set_sphere = SphereLevelSet(radius);
    Vector3<double> translation_sphere = {0.0, 0.0, 0.0};
    math::RigidTransform<double> pose_sphere =
                            math::RigidTransform<double>(translation_sphere);
    multibody::SpatialVelocity<double> velocity_sphere;
    velocity_sphere.translational() = Vector3<double>{v_strength, v_strength, v_strength};
    velocity_sphere.rotational() = Vector3<double>{0.0, 0.0, 0.0};

    double E = 1e6;
    double nu = 0.4;
    std::unique_ptr<CorotatedElasticModel> elastoplastic_model
            = std::make_unique<CorotatedElasticModel>(E, nu);
    MPMDriver::MaterialParameters m_param_sphere{
                                                std::move(elastoplastic_model),
                                                1000,
                                                velocity_sphere,
                                                1
                                                };

    // Initialize another sphere
    SphereLevelSet level_set_sphere2 = SphereLevelSet(radius);
    math::RigidTransform<double> pose_sphere2 =
                            math::RigidTransform<double>(translation_sphere2);
    multibody::SpatialVelocity<double> velocity_sphere2;
    velocity_sphere2.translational() = Vector3<double>{-v_strength, -v_strength, -v_strength};
    velocity_sphere2.rotational() = Vector3<double>{0.0, 0.0, 0.0};

    std::unique_ptr<CorotatedElasticModel> elastoplastic_model2
            = std::make_unique<CorotatedElasticModel>(E, nu);
    MPMDriver::MaterialParameters m_param_sphere2{
                                                std::move(elastoplastic_model2),
                                                1000,
                                                velocity_sphere2,
                                                1
                                                };

    driver->InitializeKinematicCollisionObjects(std::move(objects));
    driver->InitializeParticles(level_set_sphere, pose_sphere,
                                std::move(m_param_sphere));
    driver->InitializeParticles(level_set_sphere2, pose_sphere2,
                                std::move(m_param_sphere2));
    driver->DoTimeStepping();

    return 0;
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake

int main() {
    return drake::multibody::mpm::DoMain();
}
