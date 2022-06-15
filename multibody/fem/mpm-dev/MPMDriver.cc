#include "drake/multibody/fem/mpm-dev/MPMDriver.h"

namespace drake {
namespace multibody {
namespace mpm {

MPMDriver::MPMDriver(MPMParameters param):
                                param_(std::move(param)),
                                grid_(param.solver_param.num_gridpt_1D,
                                      param.solver_param.h,
                                      param.solver_param.bottom_corner),
                                gravitational_force_(param.physical_param.g) {
    DRAKE_DEMAND(param.solver_param.endtime >= 0.0);
    DRAKE_DEMAND(param.solver_param.dt > 0.0);
}

void MPMDriver::InitializeBoundaryConditions(
                        std::vector<BoundaryCondition::Boundary> boundaries) {
    boundary_condition_ = BoundaryCondition(std::move(boundaries));
}

void MPMDriver::DoTimeStepping() {
    int step = 0;
    int io_step = 0;
    double endtime = param_.solver_param.endtime;
    double dt = param_.solver_param.dt;
    bool is_last_step = false;
    // Advance time steps until endtime
    for (double t = 0; t < endtime; t += dt) {
        is_last_step = (endtime - t <= dt);
        // If at the final timestep, modify the timestep size to match endtime
        if (endtime - t < dt) {
            dt = endtime - t;
        }
        // Check the CFL condition
        checkCFL(dt);
        AdvanceOneTimeStep();
        step++;
        if ((t >= io_step*param_.io_param.write_interval) || (is_last_step)) {
            std::cout << "==== MPM Step " << step << " iostep " << io_step <<
                         ", at t = " << t+dt << std::endl;
            WriteParticlesToBgeo(io_step++);
        }
    }
}

void MPMDriver::InitializeParticles(const AnalyticLevelSet& level_set,
                                    const math::RigidTransform<double>& pose,
                                    MaterialParameters m_param) {
    DRAKE_DEMAND(m_param.density > 0.0);
    DRAKE_DEMAND(m_param.min_num_particles_per_cell >= 1);

    double h = param_.solver_param.h;
    const std::array<Vector3<double>, 2> bounding_box =
                                    level_set.get_bounding_box();

    // Distances between generated particles are at at least sample_r apart, and
    // there are at least min_num_particles_per_cell particles per cell. r =
    // argmax(⌊h/r⌋)^3 >= min_num_particles_per_cell, in other words, if we pick
    // particles located at the grid with grid size r, there are at least
    // min_num_particles_per_cell particles in a cell with size h.
    double sample_r =
            h/(std::cbrt(m_param.min_num_particles_per_cell)+1);
    Vector3<double> init_v = m_param.initial_velocity;
    std::array<double, 3> xmin = {bounding_box[0][0], bounding_box[0][1],
                                  bounding_box[0][2]};
    std::array<double, 3> xmax = {bounding_box[1][0], bounding_box[1][1],
                                  bounding_box[1][2]};
    // Generate sample particles in the reference frame (centered at the origin
    // with canonical basis e_i)
    std::vector<Vector3<double>> particles_sample_positions =
        thinks::PoissonDiskSampling<double, 3, Vector3<double>>(sample_r,
                                                                xmin, xmax);

    // Pick out sampled particles that are in the object
    int num_samples = particles_sample_positions.size();
    std::vector<Vector3<double>> particles_positions;
    for (int p = 0; p < num_samples; ++p) {
        const Vector3<double>& xp_ref = particles_sample_positions[p];
        if (level_set.InInterior(xp_ref)) {
            particles_positions.emplace_back(pose*xp_ref);
        }
    }

    int num_particles = particles_positions.size();
    // We assume every particle have the same volume and mass
    double reference_volume_p = level_set.get_volume()/num_particles;
    double init_m = m_param.density*reference_volume_p;

    // Add particles
    for (int p = 0; p < num_particles; ++p) {
        const Vector3<double>& xp = particles_positions[p];
        particles_.AddParticle(xp, init_v, init_m, reference_volume_p,
                               Matrix3<double>::Identity(),
                               Matrix3<double>::Identity(),
                               m_param.corotated_model);
    }
}

void MPMDriver::WriteParticlesToBgeo(int io_step) {
    std::string output_filename = param_.io_param.output_directory + "/"
                                + param_.io_param.case_name
                                + std::to_string(io_step) + ".bgeo";
    internal::WriteParticlesToBgeo(output_filename, particles_.get_positions(),
                                                    particles_.get_velocities(),
                                                    particles_.get_masses());
}

void MPMDriver::checkCFL(double dt) {
    for (const auto& v : particles_.get_velocities()) {
        if (!(std::max({std::abs(dt*v(0)),
                        std::abs(dt*v(1)),
                        std::abs(dt*v(2))}) <= grid_.get_h())) {
            throw std::runtime_error("CFL condition violation");
        }
    }
}

void MPMDriver::AdvanceOneTimeStep() {
    double dt = param_.solver_param.dt;

    // Update Stresses on particles
    particles_.UpdateKirchhoffStresses();

    // Set up the transfer routines (Preallocations, sort the particles)
    mpm_transfer_.SetUpTransfer(grid_, &particles_);

    // Main Algorithm:
    // P2G
    mpm_transfer_.TransferParticlesToGrid(particles_, &grid_);

    // Update grid velocity
    grid_.UpdateVelocity(dt);

    // Apply gravitational force and enforce boundary condition
    gravitational_force_.ApplyGravitationalForces(dt, &grid_);
    grid_.EnforceBoundaryCondition(boundary_condition_);

    // G2P
    mpm_transfer_.TransferGridToParticles(grid_, dt, &particles_);

    // Advect particles
    particles_.AdvectParticles(dt);
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
