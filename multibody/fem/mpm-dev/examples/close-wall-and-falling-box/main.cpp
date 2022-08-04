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
#include "drake/multibody/fem/mpm-dev/StvkHenckyWithVonMisesModel.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace multibody {
namespace mpm {

void velocity_field(Vector3<double>, double, Vector3<double>*) { }

int DoMain() {
    double h = 0.05;
    // double tw = 0.0;
    double tw = -0.5;
    // double h = 0.1;
    MPMParameters::PhysicalParameters p_param {
        {0.0, 0.0, -9.81},                      // Gravitational acceleration
        velocity_field,
    };

    MPMParameters::SolverParameters s_param {
        2e-0,                                  // End time
        5e-4,                                  // Time step size
        h,                                     // Grid size
        0.25,                                  // CFL
    };

    MPMParameters::IOParameters io_param {
        "mpm-test",                            // case name
        "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/close-wall-and-falling-box/outputs-nopenetrate",       // output directory name
        0.04,                                  // Interval of outputting
    };

    MPMParameters param {p_param, s_param, io_param};
    auto driver = std::make_unique<MPMDriver>(std::move(param));

    KinematicCollisionObjects objects = KinematicCollisionObjects();

    double wall_mu = 0.8;
    Vector3<double> wall_normal = {0.0, 0.0, 1.0};

    // Initialize the bottom wall
    SpatialVelocity<double> zero_velocity;
    zero_velocity.SetZero();
    std::unique_ptr<AnalyticLevelSet> wall_level_set =
                            std::make_unique<HalfSpaceLevelSet>(wall_normal);
    Vector3<double> wall_translation = {0.0, 0.0, tw};
    math::RigidTransform<double> wall_pose =
                            math::RigidTransform<double>(wall_translation);
    std::unique_ptr<SpatialVelocityTimeDependent> wall_velocity_ptr =
        std::make_unique<SpatialVelocityTimeDependent>(zero_velocity);
    objects.AddCollisionObject(std::move(wall_level_set),
                               std::move(wall_pose),
                               std::move(wall_velocity_ptr), wall_mu);


    Vector3<double> xscale0 = {0.2, 0.2, 0.2};
    BoxLevelSet level_set_box0 = BoxLevelSet(xscale0);
    Vector3<double> translation_box0 = {0.0, 0.0, 0.0};
    math::RigidTransform<double> pose_box0 =
                            math::RigidTransform<double>(translation_box0);
    multibody::SpatialVelocity<double> velocity_box0;
    velocity_box0.translational() = Vector3<double>{0.0, 0.0, 0.0};
    velocity_box0.rotational() = Vector3<double>{0.0, 0.0, 0.0};

    double E = 1e5;
    double nu = 0.4;
    // double tau = 0.01*E;
    // std::unique_ptr<StvkHenckyWithVonMisesModel> elastoplastic_model0
    //         = std::make_unique<StvkHenckyWithVonMisesModel>(E, nu,
    //                                                         tau);
    std::unique_ptr<CorotatedElasticModel> elastoplastic_model0
            = std::make_unique<CorotatedElasticModel>(E, nu);
    MPMDriver::MaterialParameters m_param_box0{
                                                std::move(elastoplastic_model0),
                                                2000,
                                                velocity_box0,
                                                1
                                                };

    driver->InitializeKinematicCollisionObjects(std::move(objects));
    driver->InitializeParticles(level_set_box0, pose_box0,
                                std::move(m_param_box0));
    driver->DoTimeStepping();

    return 0;
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake

int main() {
    return drake::multibody::mpm::DoMain();
}
