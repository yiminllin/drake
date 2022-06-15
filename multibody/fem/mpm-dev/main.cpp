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
#include "drake/multibody/fem/mpm-dev/MPMDriver.h"

namespace drake {
namespace multibody {
namespace mpm {

int DoMain() {
    MPMParameters::PhysicalParameters p_param {
        {0.0, 0.0, -9.81}                      // Gravitational acceleration
    };

    MPMParameters::SolverParameters s_param {
        1e-0,                                  // End time
        2e-4,                                  // Time step size
        0.01,                                  // Grid size
        Vector3<int>(23, 23, 23),              // Number of grid points in each
                                               // direction
        Vector3<int>(-1, -1, -1),              // Bottom corner of the grid
    };

    MPMParameters::IOParameters io_param {
        "mpm-test",                            // case name
        "/home/yiminlin/Desktop/output",       // output directory name
        0.04,                                  // Interval of outputting
    };

    MPMParameters param {p_param, s_param, io_param};
    auto driver = std::make_unique<MPMDriver>(std::move(param));

    BoundaryCondition::Boundary b0 = {0.5, {{1, 0, 1}, {0.1, 0, 0}}};
    BoundaryCondition::Boundary b1 = {0.1, {{-1, 0, 1}, {0.1, 0, 0}}};
    std::vector<BoundaryCondition::Boundary> boundaries = {b0, b1};

    // Initialize a box
    Vector3<double> xscale = {0.02, 0.02, 0.02};
    BoxLevelSet level_set_box = BoxLevelSet(xscale);
    Vector3<double> translation_box = {0.18, 0.1, 0.18};
    math::RigidTransform<double> pose_box =
                            math::RigidTransform<double>(translation_box);
    MPMDriver::MaterialParameters m_param_box { {8e4, 0.49},      // Corotated
                                                1272,             // Density
                                                {0.0, 0.0, 0.0},  // Velocity
                                                1,                // min number
                                                                  // particles
                                                                  // per cell
                                              };

    // Initialize a sphere
    double radius = 0.02;
    SphereLevelSet level_set_sphere = SphereLevelSet(radius);
    Vector3<double> translation_sphere = {0.02, 0.1, 0.18};
    math::RigidTransform<double> pose_sphere =
                            math::RigidTransform<double>(translation_sphere);
    MPMDriver::MaterialParameters m_param_sphere { {8e1, 0.49},
                                                   800,
                                                   {0.0, 0.0, 0.0},
                                                   1
                                                 };

    driver->InitializeBoundaryConditions(std::move(boundaries));
    driver->InitializeParticles(level_set_box, pose_box,
                                std::move(m_param_box));
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
