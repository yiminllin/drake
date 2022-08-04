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

void velocity_field(Vector3<double>, double, Vector3<double>*) { }


int DoMain() {
    // double wall_mu = 0.0;
    double wall_mu = 0.27;
    double angle = M_PI/12;
    Vector3<double> wall_normal = {sin(angle), 0.0, cos(angle)};

    MPMParameters::PhysicalParameters p_param {
        {0.0, 0.0, -9.81},                     // Gravitational acceleration
        velocity_field,
    };

    MPMParameters::SolverParameters s_param {
        // 3e-2,                                  // End time
        10e-0,                                  // End time
        1e-4,                                  // Time step size
        // 0.02,                                   // Grid size
        0.01,                                   // CFL
        0.2,
    };

    MPMParameters::IOParameters io_param {
        "mpm-test",                            // case name
        "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/slide-under-slope/outputs-mu.27",       // output directory name
        0.04,                                  // Interval of outputting
    };

    MPMParameters param {p_param, s_param, io_param};

    auto driver = std::make_unique<MPMDriver>(std::move(param));

    KinematicCollisionObjects objects = KinematicCollisionObjects();

    // Initialize the bottom wall
    SpatialVelocity<double> zero_velocity;
    zero_velocity.SetZero();
    std::unique_ptr<AnalyticLevelSet> wall_level_set =
                            std::make_unique<HalfSpaceLevelSet>(wall_normal);
    Vector3<double> wall_translation = {0.0, 0.0, 0.0};
    math::RigidTransform<double> wall_pose =
                            math::RigidTransform<double>(wall_translation);
    std::unique_ptr<SpatialVelocityTimeDependent> wall_velocity_ptr =
        std::make_unique<SpatialVelocityTimeDependent>(zero_velocity);
    objects.AddCollisionObject(std::move(wall_level_set),
                               std::move(wall_pose),
                               std::move(wall_velocity_ptr), wall_mu);

    // Initialize a box
    double l = 0.02;
    Vector3<double> xscale = {l, l, l};
    BoxLevelSet level_set_box = BoxLevelSet(xscale);
    Vector3<double> translation_box = wall_normal*l;
    math::RollPitchYaw<double> rpw_box = {0.0, angle, 0.0};
    math::RigidTransform<double> pose_box =
                        math::RigidTransform<double>(rpw_box, translation_box);
    multibody::SpatialVelocity<double> velocity_box;
    velocity_box.translational() = Vector3<double>::Zero();
    velocity_box.rotational() = Vector3<double>{0.0, 0.0, 0.0};

    double E = 1e5;
    double nu = 0.0;
    std::unique_ptr<CorotatedElasticModel> elastoplastic_model
            = std::make_unique<CorotatedElasticModel>(E, nu);
    MPMDriver::MaterialParameters m_param_box{
                                                std::move(elastoplastic_model),
                                                1200,
                                                velocity_box,
                                                10,
                                                };

    driver->InitializeKinematicCollisionObjects(std::move(objects));
    driver->InitializeParticles(level_set_box, pose_box,
                                std::move(m_param_box));
    driver->DoTimeStepping();

    return 0;
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake

int main() {
    return drake::multibody::mpm::DoMain();
}
