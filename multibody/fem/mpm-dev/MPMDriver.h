#pragma once

#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/mpm-dev/AnalyticLevelSet.h"
#include "drake/multibody/fem/mpm-dev/CorotatedModel.h"
#include "drake/multibody/fem/mpm-dev/GravitationalForce.h"
#include "drake/multibody/fem/mpm-dev/Grid.h"
#include "drake/multibody/fem/mpm-dev/MPMParameters.h"
#include "drake/multibody/fem/mpm-dev/MPMTransfer.h"
#include "drake/multibody/fem/mpm-dev/Particles.h"
#include "drake/multibody/fem/mpm-dev/particles_to_bgeo.h"
#include "drake/multibody/fem/mpm-dev/poisson_disk_sampling.h"

namespace drake {
namespace multibody {
namespace mpm {

class MPMDriver {
 public:
    // Struct containing parameters describing the objects to be modelled in MPM
    struct MaterialParameters {
        // Constitutive model of the object
        CorotatedModel corotated_model;
        // @pre density is positive
        // Density and the initial velocity of the object, we assume the object
        // has uniform density and velocity.
        double density;
        Vector3<double> initial_velocity;
        // User defined parameter to control the minimum number of particles per
        // grid cell.
        int min_num_particles_per_cell;
    };

    explicit MPMDriver(MPMParameters param);

    // Initialize the boundary condition by the boundaries information
    void InitializeBoundaryConditions(std::vector<BoundaryCondition::Boundary>
                                                                    boundaries);

    // Initialize particles' positions with Poisson disk sampling. The object's
    // level set in the physical frame is the given level set in the reference
    // frame transformed by pose. We assume every particles have equal reference
    // volumes, then we can initialize particles' masses with the given constant
    // density, Finally, we initialize the velocities of particles with the
    // constant given velocity.
    void InitializeParticles(const AnalyticLevelSet& level_set,
                             const math::RigidTransform<double>& pose,
                             MaterialParameters param);

    // Symplectic Euler time stepping till endtime with dt
    void DoTimeStepping();

 private:
    friend class MPMDriverTest;

    // @throw if the given dt doesn't violate the CFL condition
    // CFL condition: max dt*|velocity| <= h, | |: pointwise absoluate value
    // dt: timestep size, h: grid size
    void checkCFL(double dt);

    // Advance MPM simulation by a single time step. Assuming both grid and
    // particles' state are at time n, a single time step involves a P2G
    // transfer, grid velocities update, a G2P transfer, and particles'
    // velocities update.
    void AdvanceOneTimeStep();

    // For every write_interval, write particles' information to
    // output_directory/case_name($step).bgeo
    void WriteParticlesToBgeo(int step);

    MPMParameters param_;
    Particles particles_;
    Grid grid_;
    MPMTransfer mpm_transfer_;
    GravitationalForce gravitational_force_;
    BoundaryCondition boundary_condition_;
};  // class MPMDriver

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
