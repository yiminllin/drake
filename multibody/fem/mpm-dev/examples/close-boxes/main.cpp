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
    double h = 0.1;
    double dist = 3.0*h;
    MPMParameters::PhysicalParameters p_param {
        {0.0, 0.0, 0.0},                      // Gravitational acceleration
        velocity_field,
    };

    MPMParameters::SolverParameters s_param {
        2e-0,                                  // End time
        5e-4,                                  // Time step size
        h,                                     // Grid size
        0.75,                                  // CFL
    };

    MPMParameters::IOParameters io_param {
        "mpm-test",                            // case name
        "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/close-boxes/outputs-3h",       // output directory name
        0.04,                                  // Interval of outputting
    };

    MPMParameters param {p_param, s_param, io_param};
    auto driver = std::make_unique<MPMDriver>(std::move(param));

    KinematicCollisionObjects objects = KinematicCollisionObjects();

    Vector3<double> xscale0 = {0.5, 0.5, 0.5};
    BoxLevelSet level_set_box0 = BoxLevelSet(xscale0);
    Vector3<double> translation_box0 = {0.0, 0.0, 0.0};
    math::RigidTransform<double> pose_box0 =
                            math::RigidTransform<double>(translation_box0);
    multibody::SpatialVelocity<double> velocity_box0;
    velocity_box0.translational() = Vector3<double>{0.0, 0.0, -1.0};
    velocity_box0.rotational() = Vector3<double>{0.0, 0.0, 0.0};

    double E = 1e5;
    double nu = 0.4;
    std::unique_ptr<CorotatedElasticModel> elastoplastic_model0
            = std::make_unique<CorotatedElasticModel>(E, nu);
    MPMDriver::MaterialParameters m_param_box0{
                                                std::move(elastoplastic_model0),
                                                2000,
                                                velocity_box0,
                                                1
                                                };

    Vector3<double> xscale1 = {0.5, 0.5, 0.5};
    BoxLevelSet level_set_box1 = BoxLevelSet(xscale1);
    Vector3<double> translation_box1 = {1.0+dist, 0.0, -2.0};
    math::RigidTransform<double> pose_box1 =
                            math::RigidTransform<double>(translation_box1);
    multibody::SpatialVelocity<double> velocity_box1;
    velocity_box1.translational() = Vector3<double>{0.0, 0.0, 1.0};
    velocity_box1.rotational() = Vector3<double>{0.0, 0.0, 0.0};

    std::unique_ptr<CorotatedElasticModel> elastoplastic_model1
            = std::make_unique<CorotatedElasticModel>(E, nu);
    MPMDriver::MaterialParameters m_param_box1{
                                                std::move(elastoplastic_model1),
                                                2000,
                                                velocity_box1,
                                                1
                                                };

    driver->InitializeKinematicCollisionObjects(std::move(objects));
    driver->InitializeParticles(level_set_box0, pose_box0,
                                std::move(m_param_box0));
    driver->InitializeParticles(level_set_box1, pose_box1,
                                std::move(m_param_box1));
    driver->DoTimeStepping();

    return 0;
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake

int main() {
    return drake::multibody::mpm::DoMain();
}
