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

void velocity_field(Vector3<double>, double, Vector3<double>*) {
}

multibody::SpatialVelocity<double> left_hand_velocity(double t) {
    multibody::SpatialVelocity<double> wall_velocity;
    wall_velocity.rotational() = Vector3<double>(0.0, 0.0, 0.0);
    if (t < 0.6) {
        wall_velocity.translational() = Vector3<double>( 0.5, 0.0, 0.0);
    } else {
        wall_velocity.translational() = Vector3<double>(-1.0, 0.0, 0.0);
    }
    return wall_velocity;
}

multibody::SpatialVelocity<double> right_hand_velocity(double t) {
    multibody::SpatialVelocity<double> wall_velocity;
    wall_velocity.rotational() = Vector3<double>(0.0, 0.0, 0.0);
    if (t < 0.6) {
        wall_velocity.translational() = Vector3<double>(-0.5, 0.0, 0.0);
    } else {
        wall_velocity.translational() = Vector3<double>(1.0, 0.0, 0.0);
    }
    return wall_velocity;
}



int DoMain() {
    double E = 1e8;
    double nu = 0.49;

    MPMParameters::PhysicalParameters p_param {
        {0.0, 0.0, 0.0},                      // Gravitational acceleration
        velocity_field,
    };

    MPMParameters::SolverParameters s_param {
        2e-0,                                  // End time
        5e-4,                                  // Time step size
        0.075,                                  // Grid size
        // 0.2,                                  // Grid size
        0.2,                                  // CFL
    };

    MPMParameters::IOParameters io_param {
        "mpm-test",                            // case name
        "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/compress-box/outputs-fixnu.25-E1e8",       // output directory name
        0.04,                                  // Interval of outputting
    };

    MPMParameters param {p_param, s_param, io_param};
    auto driver = std::make_unique<MPMDriver>(std::move(param));

    KinematicCollisionObjects objects = KinematicCollisionObjects();

    // Initialize the left wall
    std::unique_ptr<SpatialVelocityTimeDependent> left_hand_velocity_ptr =
        std::make_unique<SpatialVelocityTimeDependent>(left_hand_velocity);
    double left_hand_mu = 100.0;
    Vector3<double> left_hand_xscale = {0.1, 0.5, 0.5};
    std::unique_ptr<AnalyticLevelSet> left_hand_level_set =
                            std::make_unique<BoxLevelSet>(left_hand_xscale);
    Vector3<double> left_hand_translation = {-0.8, 0.0, 0.0};
    math::RigidTransform<double> left_hand_pose =
                            math::RigidTransform<double>(left_hand_translation);
    objects.AddCollisionObject(std::move(left_hand_level_set), std::move(left_hand_pose),
                               std::move(left_hand_velocity_ptr), left_hand_mu);

    // Initialize the right wall
    std::unique_ptr<SpatialVelocityTimeDependent> right_hand_velocity_ptr =
        std::make_unique<SpatialVelocityTimeDependent>(right_hand_velocity);
    double right_hand_mu = 100.0;
    Vector3<double> right_hand_xscale = {0.1, 0.5, 0.5};
    std::unique_ptr<AnalyticLevelSet> right_hand_level_set =
                            std::make_unique<BoxLevelSet>(right_hand_xscale);
    Vector3<double> right_hand_translation = {0.8, 0.0, 0.0};
    math::RigidTransform<double> right_hand_pose =
                            math::RigidTransform<double>(right_hand_translation);
    objects.AddCollisionObject(std::move(right_hand_level_set), std::move(right_hand_pose),
                               std::move(right_hand_velocity_ptr), right_hand_mu);


    // Initialize a box
    Vector3<double> xscale = {0.6, 0.3, 0.3};
    BoxLevelSet level_set_box = BoxLevelSet(xscale);
    Vector3<double> translation_box = {0.0, 0.0, 0.0};
    math::RigidTransform<double> pose_box =
                            math::RigidTransform<double>(translation_box);
    multibody::SpatialVelocity<double> velocity_box;
    velocity_box.translational() = Vector3<double>::Zero();
    velocity_box.rotational() = Vector3<double>{0.0, 0.0, 0.0};

    std::unique_ptr<CorotatedElasticModel> elastoplastic_model
            = std::make_unique<CorotatedElasticModel>(E, nu);
    MPMDriver::MaterialParameters m_param_box{
                                                std::move(elastoplastic_model),
                                                2000,
                                                velocity_box,
                                                1
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
