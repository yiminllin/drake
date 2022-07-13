#include "drake/multibody/fem/mpm-dev/MPMTransfer.h"

namespace drake {
namespace multibody {
namespace mpm {

void MPMTransfer::SetUpTransfer(const Grid& grid, Particles* particles) {
    SortParticles(grid, particles);
    UpdateBasisAndGradientParticles(grid, *particles);
    // TODO(yiminlin.tri): Dp_inv_ is hardcoded for quadratic B-Spline
    // The values of Dp_inv_ are different for different B-Spline bases
    // https://www.math.ucla.edu/~jteran/papers/JSSTS15.pdf
    Dp_inv_ = 4.0/(grid.get_h()*grid.get_h());
}

void MPMTransfer::TransferParticlesToGrid(const Particles& particles,
                                          Grid* grid) {
    int p_start, p_end, bi, bj, bk, idx_local;
    double mass_p, ref_volume_p;
    // Local sum of states m_i v_i f_i on the grid points
    std::array<GridState, 27> local_pad;
    // Positions of grid points in the batch
    std::array<Vector3<double>, 27> batch_positions;

    // Clear grid states
    grid->ResetStates();

    // For each batch of particles
    p_start = 0;
    for (const auto& [batch_index_flat, batch_index_3d] : grid->get_indices()) {
        if (batch_sizes_[batch_index_flat] != 0) {
            p_end = p_start + batch_sizes_[batch_index_flat];

            // Preallocate positions at grid points in the current batch on a
            // local array
            bi = batch_index_3d[0];
            bj = batch_index_3d[1];
            bk = batch_index_3d[2];
            for (int a = -1; a <= 1; ++a) {
            for (int b = -1; b <= 1; ++b) {
            for (int c = -1; c <= 1; ++c) {
                idx_local = (a+1) + 3*(b+1) + 9*(c+1);
                batch_positions[idx_local] =
                                        grid->get_position(bi+a, bj+b, bk+c);
            }
            }
            }

            // Clear local scratch pad
            for (auto& s : local_pad) { s.reset(); }

            // For each particle in the batch (Assume particles are sorted with
            // respect to the batch index), accmulate masses, momemtum, and
            // forces into grid points affected by the particle.
            for (int p = p_start; p < p_end; ++p) {
                mass_p = particles.get_mass(p);
                ref_volume_p = particles.get_reference_volume(p);
                // The affine matrix Cp = Bp * Dp^-1
                const Matrix3<double> C_p = particles.get_B_matrix(p)*Dp_inv_;
                AccumulateGridStatesOnBatch(p, mass_p, ref_volume_p,
                                            particles.get_position(p),
                                            particles.get_velocity(p),
                                            C_p,
                                            particles.get_kirchhoff_stress(p),
                                            batch_positions, &local_pad);
            }

            // Put sums of local scratch pads to grid
            WriteBatchStateToGrid(batch_index_3d, local_pad, grid);

            p_start = p_end;
        }
    }

    // Calculate grid velocities v_i by (mv)_i / m_i
    grid->RescaleVelocities();
}

void MPMTransfer::TransferGridToParticles(const Grid& grid, double dt,
                                          Particles* particles) {
    DRAKE_ASSERT(dt > 0.0);
    int bi, bj, bk, idx_local;
    int p_start, p_end;
    // A local array holding positions and velocities x^{n+1}_i, v^{n+1}_i at a
    // batch
    std::array<BatchState, 27> batch_states;

    // For each batch of particles
    p_start = 0;
    for (const auto& [batch_index_flat, batch_index_3d] : grid.get_indices()) {
        p_end = p_start + batch_sizes_[batch_index_flat];

        // Preallocate positions and velocities at grid points in the current
        // batch on a local array
        bi = batch_index_3d[0];
        bj = batch_index_3d[1];
        bk = batch_index_3d[2];
        for (int a = -1; a <= 1; ++a) {
        for (int b = -1; b <= 1; ++b) {
        for (int c = -1; c <= 1; ++c) {
            idx_local = (a+1) + 3*(b+1) + 9*(c+1);
            batch_states[idx_local].position =
                                            grid.get_position(bi+a, bj+b, bk+c);
            batch_states[idx_local].velocity =
                                            grid.get_velocity(bi+a, bj+b, bk+c);
        }
        }
        }

        // For each particle in the batch (Assume particles are sorted with
        // respect to the batch index), update the particles' states
        for (int p = p_start; p < p_end; ++p) {
            UpdateParticleStates(batch_states, dt, p, particles);
        }

        p_start = p_end;
    }
}

// TODO(yiminlin.tri): We assume particles all lies in the grid
// Also may be a good idea to remove this routine's dependency on the grid,
// this need future refactoring
void MPMTransfer::SortParticles(const Grid& grid, Particles* particles) {
    Vector3<int> batch_idx_3D;
    int num_particles = particles->get_num_particles();
    // A temporary array storing the particle index permutation after sorting
    std::vector<size_t> sorted_indices(num_particles);
    // A temporary array storing the batch index correspond to each particle
    std::vector<int> batch_indices(num_particles);
    // Initialize batch_size to be 0 for every batch
    batch_sizes_.resize(grid.get_num_gridpt());
    fill(batch_sizes_.begin(), batch_sizes_.end(), 0);

    // Preallocate the indices of batches, and check particles out of bounds
    // error
    for (int p = 0; p < num_particles; ++p) {
        batch_idx_3D = CalcBatchIndex(particles->get_position(p), grid.get_h());
        batch_indices[p] = grid.Reduce3DIndex(batch_idx_3D(0), batch_idx_3D(1),
                                              batch_idx_3D(2));
        for (int a = -1; a <= 1; ++a) {
        for (int b = -1; b <= 1; ++b) {
        for (int c = -1; c <= 1; ++c) {
        if (!grid.in_index_range(batch_idx_3D[0]+a, batch_idx_3D[1]+b,
                                 batch_idx_3D[2]+c)) {
            throw std::logic_error("Particles out of bound");
        }
        }
        }
        }
    }

    // Accumulate batch sizes
    for (int p = 0; p < num_particles; ++p) {
        ++batch_sizes_[batch_indices[p]];
    }

    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
       [&grid, &batch_indices](size_t i1, size_t i2) {
                                return batch_indices[i1] < batch_indices[i2];});

    // Reorder the particles
    particles->Reorder(sorted_indices);
}

void MPMTransfer::UpdateBasisAndGradientParticles(const Grid& grid,
                                                  const Particles& particles) {
    int num_particles, p_start, p_end, count;
    double h;
    Vector3<int> num_gridpt_1D = grid.get_num_gridpt_1D();
    Vector3<int> bottom_corner = grid.get_bottom_corner();
    std::vector<BSpline> bases(grid.get_num_gridpt());
    num_particles = particles.get_num_particles();
    h = grid.get_h();

    bases_val_particles_.reserve(num_particles);
    bases_grad_particles_.reserve(num_particles);

    // Define the bases vector, whose ith entry is the BSpline basis correspond
    // to ith BSpline basis.
    count = 0;
    for (int k = bottom_corner(2); k < bottom_corner(2) + num_gridpt_1D(2);
                                                                        ++k) {
    for (int j = bottom_corner(1); j < bottom_corner(1) + num_gridpt_1D(1);
                                                                        ++j) {
    for (int i = bottom_corner(0); i < bottom_corner(0) + num_gridpt_1D(0);
                                                                        ++i) {
        bases[count++] = BSpline(h, grid.get_position(i, j, k));
    }
    }
    }

    // For each batch of particles
    p_start = 0;
    for (const auto& [batch_index_flat, batch_index_3d] : grid.get_indices()) {
        p_end = p_start + batch_sizes_[batch_index_flat];
        // For each particle in the batch (Assume particles are sorted with
        // respect to the batch index), update basis evaluations
        for (int p = p_start; p < p_end; ++p) {
            const Vector3<double>& xp = particles.get_position(p);
            EvalBasisOnBatch(p, xp, grid, batch_index_3d, bases);
        }
        p_start = p_end;
    }
}

void MPMTransfer::EvalBasisOnBatch(int p, const Vector3<double>& xp,
                                   const Grid& grid,
                                   const Vector3<int>& batch_index_3d,
                                   const std::vector<BSpline>& bases) {
    int bi = batch_index_3d[0];
    int bj = batch_index_3d[1];
    int bk = batch_index_3d[2];
    int idx_local;
    Vector3<int> grid_index;

    for (int a = -1; a <= 1; ++a) {
    for (int b = -1; b <= 1; ++b) {
    for (int c = -1; c <= 1; ++c) {
        grid_index(0) = bi+a;
        grid_index(1) = bj+b;
        grid_index(2) = bk+c;
        idx_local = (a+1) + 3*(b+1) + 9*(c+1);
        // For each particle in the batch (Assume particles are sorted with
        // respect to the batch index), update basis evaluations
        std::tie(bases_val_particles_[p][idx_local],
                bases_grad_particles_[p][idx_local]) =
        bases[grid.Reduce3DIndex(grid_index)].EvalBasisAndGradient(xp);
    }
    }
    }
}

void MPMTransfer::AccumulateGridStatesOnBatch(int p, double m_p,
                                double reference_volume_p,
                                const Vector3<double>& x_p,
                                const Vector3<double>& v_p,
                                const Matrix3<double>& C_p,
                                const Matrix3<double>& tau_p,
                                const std::array<Vector3<double>, 27>&
                                                    batch_positions,
                                std::array<GridState, 27>* local_pad) {
    int idx_local;
    double Ni_p;

    // Accumulate on local scratch pads
    for (int a = -1; a <= 1; ++a) {
    for (int b = -1; b <= 1; ++b) {
    for (int c = -1; c <= 1; ++c) {
        idx_local = (a+1) + 3*(b+1) + 9*(c+1);
        Ni_p = bases_val_particles_[p][idx_local];
        const Vector3<double>& x_i = batch_positions[idx_local];
        const Vector3<double>& gradNi_p  = bases_grad_particles_[p][idx_local];
        // For each particle in the batch (Assume particles are sorted with
        // respect to the batch index), update basis evaluations
        GridState& state_i = (*local_pad)[idx_local];
        double m_ip = m_p*Ni_p;
        state_i.mass += m_ip;
        // PIC update:
        // state_i.velocity += m_ip*v_p;
        // TODO(yiminlin.tri): This also conserves angular momentum, but is
        //                     using a incorrect linearization. May want to
        //                     look at it further?
        // state_i.velocity += m_ip*v_p+m_p*B_p*gradNi_p;
        // APIC update:
        state_i.velocity += m_ip*(v_p+C_p*(x_i-x_p));
        state_i.force += -reference_volume_p*tau_p*gradNi_p;
    }
    }
    }
}

void MPMTransfer::WriteBatchStateToGrid(const Vector3<int>& batch_index_3d,
                                const std::array<GridState, 27>& local_pad,
                                Grid* grid) {
    int bi = batch_index_3d[0];
    int bj = batch_index_3d[1];
    int bk = batch_index_3d[2];
    int idx_local;
    Vector3<int> grid_index;

    // Put local scratch pad states into the grid
    for (int a = -1; a <= 1; ++a) {
    for (int b = -1; b <= 1; ++b) {
    for (int c = -1; c <= 1; ++c) {
        grid_index(0) = bi+a;
        grid_index(1) = bj+b;
        grid_index(2) = bk+c;
        idx_local = (a+1) + 3*(b+1) + 9*(c+1);
        const GridState& state_i = local_pad[idx_local];
        grid->AccumulateMass(grid_index, state_i.mass);
        grid->AccumulateVelocity(grid_index, state_i.velocity);
        grid->AccumulateForce(grid_index, state_i.force);
    }
    }
    }
}

void MPMTransfer::UpdateParticleStates(const std::array<BatchState, 27>&
                                       batch_states,
                                       double dt, int p,
                                       Particles* particles) {
    int idx_local;
    double Ni_p;
    // vp_new_i = v_i^{n+1} * N_i(x_p)
    Vector3<double> vp_new_i;
    Vector3<double> xp = particles->get_position(p);

    // Scratch vectors and matrices
    Vector3<double> vp_new = Vector3<double>::Zero();
    Matrix3<double> Bp_new = Matrix3<double>::Zero();
    Matrix3<double> grad_vp_new = Matrix3<double>::Zero();

    // For each grid points affecting the current particle
    for (int a = -1; a <= 1; ++a) {
    for (int b = -1; b <= 1; ++b) {
    for (int c = -1; c <= 1; ++c) {
        idx_local = (a+1) + 3*(b+1) + 9*(c+1);
        Ni_p = bases_val_particles_[p][idx_local];
        const Vector3<double>& gradNi_p = bases_grad_particles_[p][idx_local];
        const Vector3<double>& xi = batch_states[idx_local].position;
        const Vector3<double>& vi_new = batch_states[idx_local].velocity;
        vp_new_i = vi_new*Ni_p;
        // v_p^{n+1} = \sum v_i^{n+1} N_i(x_p)
        vp_new += vp_new_i;
        // B_p^{n+1} = \sum v_i^{n+1}*(x_i - x_p^n)^T N_i(x_p)
        Bp_new += vp_new_i*(xi-xp).transpose();
        // Accumulate grad_vp_new: F_p^{n+1} = (I + dt*grad_vp_new)*F_p^n
        grad_vp_new += vi_new*gradNi_p.transpose();
    }
    }
    }

    // Update velocities and elastic deformation gradients of the particle
    // F_p^{n+1} = (I + dt*grad_vp_new)*F_p^n
    // Note that we assume the plastic deformation gradient doesn't change
    // during the evolution.
    particles->set_elastic_deformation_gradient(p,
                        (Matrix3<double>::Identity() + dt*grad_vp_new)
                        *particles->get_elastic_deformation_gradient(p));
    particles->set_velocity(p, vp_new);
    particles->set_B_matrix(p, Bp_new);
}

Vector3<int> MPMTransfer::CalcBatchIndex(const Vector3<double>& xp, double h)
                                                                        const {
    return Vector3<int>(std::round(xp(0)/h), std::round(xp(1)/h),
                        std::round(xp(2)/h));
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
