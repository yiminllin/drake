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

void velocity_field(Vector3<double> pos, double, Vector3<double>* vel) {
    if (pos[2] > 0.36) {
        *vel = Vector3<double>{0.4, 0.0, 0.0};
    }
}

multibody::SpatialVelocity<double> left_velocity(double t) {
    multibody::SpatialVelocity<double> wall_velocity;
    wall_velocity.rotational() = Vector3<double>(0.0, 0.0, 0.0);
    if (t < 0.06) {
        wall_velocity.translational() = Vector3<double>(0.5, 0.0, 0.0);
    } else {
        wall_velocity.translational() = Vector3<double>(1.0, 0.0, 0.0);
    }
    return wall_velocity;
}

multibody::SpatialVelocity<double> right_velocity(double t) {
    multibody::SpatialVelocity<double> wall_velocity;
    wall_velocity.rotational() = Vector3<double>(0.0, 0.0, 0.0);
    if (t < 0.06) {
        wall_velocity.translational() = Vector3<double>(-0.5, 0.0, 0.0); 
    } else {
        wall_velocity.translational() = Vector3<double>(1.0, 0.0, 0.0);
    }
    return wall_velocity;
}



int DoMain() {
    double bottom_wall_mu = 5.0;
    Vector3<double> translation_box = {0.0, 0.0, 0.21};
    double E = 1e6;
    double nu = 0.1;
    MPMParameters::PhysicalParameters p_param {
        {0.0, 0.0, -9.81},                      // Gravitational acceleration
        velocity_field,
    };

    MPMParameters::SolverParameters s_param {
        // 3e-2,                                  // End time
        2e-0,                                  // End time
        1e-4,                                  // Time step size
        // 0.02,                                   // Grid size
        0.01,                                   // Grid size
        0.75,                                   // CFL
    };

    MPMParameters::IOParameters io_param {
        "mpm-test",                            // case name
        "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/stick-slip-transition/outputs-mu5",       // output directory name
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

    // // Initialize the left wall
    // double hand_mu = 10000000.0;
    // // Vector3<double> hand_xscale = {0.01, 0.05, 0.2};
    // Vector3<double> hand_xscale = {0.01, 0.05, 0.05};

    // std::unique_ptr<AnalyticLevelSet> left_hand_level_set =
    //                         std::make_unique<BoxLevelSet>(hand_xscale);
    // Vector3<double> left_hand_translation = {-0.06, 0.0, 0.4};
    // math::RigidTransform<double> left_hand_pose =
    //                         math::RigidTransform<double>(left_hand_translation);
    // std::unique_ptr<SpatialVelocityTimeDependent> left_hand_velocity_ptr =
    //     std::make_unique<SpatialVelocityTimeDependent>(left_velocity);
    // objects.AddCollisionObject(std::move(left_hand_level_set), std::move(left_hand_pose),
    //                            std::move(left_hand_velocity_ptr), hand_mu);

    // // Initialize the right wall
    // std::unique_ptr<AnalyticLevelSet> right_hand_level_set =
    //                         std::make_unique<BoxLevelSet>(hand_xscale);
    // Vector3<double> right_hand_translation = {0.06, 0.0, 0.4};
    // math::RigidTransform<double> right_hand_pose =
    //                         math::RigidTransform<double>(right_hand_translation);
    // std::unique_ptr<SpatialVelocityTimeDependent> right_hand_velocity_ptr =
    //     std::make_unique<SpatialVelocityTimeDependent>(right_velocity);
    // objects.AddCollisionObject(std::move(right_hand_level_set), std::move(right_hand_pose),
    //                            std::move(right_hand_velocity_ptr), hand_mu);

    // Initialize a box
    Vector3<double> xscale = {0.05, 0.015, 0.2};
    BoxLevelSet level_set_box = BoxLevelSet(xscale);
    math::RigidTransform<double> pose_box =
                            math::RigidTransform<double>(translation_box);
    multibody::SpatialVelocity<double> velocity_box;
    velocity_box.translational() = Vector3<double>::Zero();
    velocity_box.rotational() = Vector3<double>{0.0, 0.0, 0.0};

    std::unique_ptr<CorotatedElasticModel> elastoplastic_model
            = std::make_unique<CorotatedElasticModel>(E, nu);
    MPMDriver::MaterialParameters m_param_box{
                                                std::move(elastoplastic_model),
                                                800,
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
