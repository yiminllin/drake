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
#include "drake/multibody/fem/mpm-dev/StvkHenckyWithVonMisesModel.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace multibody {
namespace mpm {

multibody::SpatialVelocity<double> bottom_velocity(double t) {
    multibody::SpatialVelocity<double> wall_velocity;
    if (t < 0.16) {
        wall_velocity.translational() = Vector3<double>(0.0, 0.0, 0.4);
        wall_velocity.rotational() = Vector3<double>(0.0, 0.0, 0.0); 
    } else {
        wall_velocity.translational() = Vector3<double>(0.0, 0.0, 0.0);
        wall_velocity.rotational() = Vector3<double>(0.0, 0.0, M_PI);
    }
    return wall_velocity;
}

multibody::SpatialVelocity<double> top_velocity(double t) {
    multibody::SpatialVelocity<double> wall_velocity;
    if (t < 0.16) {
        wall_velocity.translational() = Vector3<double>(0.0, 0.0, -0.4); 
        wall_velocity.rotational() = Vector3<double>(0.0, 0.0, 0.0);
    } else {
        wall_velocity.translational() = Vector3<double>(0.0, 0.0, 0.0);
        wall_velocity.rotational() = Vector3<double>(0.0, 0.0, M_PI);
    }

    return wall_velocity;
}

// void velocity_field(Vector3<double> pos, double t, Vector3<double>* v) {
void velocity_field(Vector3<double> pos, double, Vector3<double>* v) {
    // Prescribed velocities on top and bottom in z direction.
    // x, y direction velocity on top and bottom is determined by the rotational
    // velocity
    Vector3<double> rot_top = {0.0, 0.0, -M_PI};
    Vector3<double> rot_bot = {0.0, 0.0,  M_PI};
    if (pos[2] > 0.06) {
        *v = pos.cross(rot_top);
        // (*v)[2] = -1.0;
    }
    if (pos[2] < -0.06) {
        *v = pos.cross(rot_bot);
        // (*v)[2] = 1.0;
    }
}


int DoMain() {
    double E = 9e4;
    double nu = 0.49;
    double yield_stress = 0.01*E;

    MPMParameters::PhysicalParameters p_param {
        {0.0, 0.0, 0.0},                     // Gravitational acceleration
        velocity_field,
    };

    MPMParameters::SolverParameters s_param {
        4e-0,                                  // End time
        4e-4,                                  // Time step size
        0.005,                                  // Grid size
        0.5,                                   // CFL
    };


    MPMParameters::IOParameters io_param {
        "mpm-test",                            // case name
        "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/twist/outputs-.01E",       // output directory name
        0.04,                                  // Interval of outputting
    };

    MPMParameters param {p_param, s_param, io_param};

    auto driver = std::make_unique<MPMDriver>(std::move(param));

    KinematicCollisionObjects objects = KinematicCollisionObjects();

    // Initialize a box
    Vector3<double> xscale = {0.02, 0.02, 0.1};
    BoxLevelSet level_set_box = BoxLevelSet(xscale);
    Vector3<double> translation_box = {0.0, 0.0, 0.0};
    math::RigidTransform<double> pose_box =
                            math::RigidTransform<double>(translation_box);
    multibody::SpatialVelocity<double> velocity_box;
    velocity_box.translational() = Vector3<double>::Zero();
    velocity_box.rotational() = Vector3<double>{0.0, 0.0, 0.0};

    std::unique_ptr<StvkHenckyWithVonMisesModel> elastoplastic_model
            = std::make_unique<StvkHenckyWithVonMisesModel>(E, nu,
                                                            yield_stress);
    MPMDriver::MaterialParameters m_param_box{
                                                std::move(elastoplastic_model),
                                                1200,
                                                velocity_box,
                                                1,
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
