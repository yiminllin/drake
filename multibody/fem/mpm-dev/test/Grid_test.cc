#include "drake/multibody/fem/mpm-dev/Grid.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/posed_half_space.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

constexpr double kEps = 4.0 * std::numeric_limits<double>::epsilon();

GTEST_TEST(GridClassTest, TestSetGet) {
    Vector3<int> num_gridpt_1D = {6, 3, 4};
    double h = 1.0;
    Vector3<int> bottom_corner  = {0, 0, 0};
    Grid grid = Grid(num_gridpt_1D, h, bottom_corner);
    double tmpscaling = 1.0;
    // Test the geometry of the grid
    EXPECT_EQ(grid.get_num_gridpt(), 72);
    EXPECT_TRUE(CompareMatrices(grid.get_num_gridpt_1D(),
                                Vector3<int>(6, 3, 4)));
    EXPECT_EQ(grid.get_h(), 1.0);
    EXPECT_TRUE(CompareMatrices(grid.get_bottom_corner(), bottom_corner));

    // Check whether the grid point positions are populated correctly
    for (int k = 0; k < 4; ++k) {
    for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 6; ++i) {
        EXPECT_TRUE(CompareMatrices(grid.get_position(i, j, k),
                                    Vector3<double>(i, j, k)));
        // Randomly put some values in
        tmpscaling = 20.0*k + 10.0*j + i;
        grid.set_mass(i, j, k, tmpscaling);
        grid.set_velocity(i, j, k, Vector3<double>(tmpscaling,
                                                  -tmpscaling,
                                                   tmpscaling));
        grid.set_force(i, j, k, Vector3<double>(-tmpscaling,
                                                 tmpscaling,
                                                -tmpscaling));
    }
    }
    }

    EXPECT_TRUE(CompareMatrices(grid.get_velocity(1, 1, 1),
                                Vector3<double>(31.0, -31.0, 31.0)));
    EXPECT_TRUE(CompareMatrices(grid.get_force(1, 1, 1),
                                Vector3<double>(-31.0, 31.0, -31.0)));
    EXPECT_EQ(grid.get_mass(1, 1, 1), 31.0);
    EXPECT_TRUE(CompareMatrices(grid.get_velocity(4, 2, 2),
                                Vector3<double>(64.0, -64.0, 64.0)));
    EXPECT_TRUE(CompareMatrices(grid.get_force(4, 2, 2),
                                Vector3<double>(-64.0, 64.0, -64.0)));
    EXPECT_EQ(grid.get_mass(4, 2, 2), 64.0);

    // Test on a new grid
    num_gridpt_1D = {3, 3, 3};
    h = 0.5;
    bottom_corner  = {-2, 2, -2};
    grid = Grid(num_gridpt_1D, h, bottom_corner);

    // Test the geometry of the grid
    EXPECT_EQ(grid.get_num_gridpt(), 27);
    EXPECT_TRUE(CompareMatrices(grid.get_num_gridpt_1D(),
                                Vector3<int>(3, 3, 3)));
    EXPECT_EQ(grid.get_h(), 0.5);
    EXPECT_TRUE(CompareMatrices(grid.get_bottom_corner(), bottom_corner));

    // Check whether the grid point positions are populated correctly
    EXPECT_TRUE(CompareMatrices(grid.get_position(-2, 2, -2),
                                Vector3<double>(-1.0, 1.0, -1.0)));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-1, 2, -2),
                                Vector3<double>(-0.5, 1.0, -1.0)));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-2, 3, -2),
                                Vector3<double>(-1.0, 1.5, -1.0)));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-1, 3, -2),
                                Vector3<double>(-0.5, 1.5, -1.0)));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-2, 2, -1),
                                Vector3<double>(-1.0, 1.0, -0.5)));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-1, 2, -1),
                                Vector3<double>(-0.5, 1.0, -0.5)));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-2, 3, -1),
                                Vector3<double>(-1.0, 1.5, -0.5)));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-1, 3, -1),
                                Vector3<double>(-0.5, 1.5, -0.5)));
    EXPECT_TRUE(CompareMatrices(grid.get_position(0, 4, 0),
                                Vector3<double>(0.0, 2.0, 0.0)));
}

GTEST_TEST(GridClassTest, TestExpand1DIndex) {
    int count;
    Vector3<int> num_gridpt_1D = {6, 3, 4};
    double h = 1.0;
    Vector3<int> bottom_corner  = {0, 0, 0};
    Grid grid = Grid(num_gridpt_1D, h, bottom_corner);

    // Check expand 1D index
    count = 0;
    for (int k = 0; k < 4; ++k) {
    for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 6; ++i) {
        EXPECT_TRUE(CompareMatrices(grid.Expand1DIndex(count++),
                                    Vector3<int>(i, j, k)));
    }
    }
    }

    // Test on a new grid
    num_gridpt_1D = {3, 3, 3};
    h = 0.5;
    bottom_corner  = {-2, 2, -2};
    grid = Grid(num_gridpt_1D, h, bottom_corner);

    // Check expand 1D index
    count = 0;
    for (int k = bottom_corner(2); k < bottom_corner(2)+num_gridpt_1D(2); ++k) {
    for (int j = bottom_corner(1); j < bottom_corner(1)+num_gridpt_1D(1); ++j) {
    for (int i = bottom_corner(0); i < bottom_corner(0)+num_gridpt_1D(0); ++i) {
        EXPECT_TRUE(CompareMatrices(grid.Expand1DIndex(count++),
                                    Vector3<int>(i, j, k)));
    }
    }
    }
}

GTEST_TEST(GridClassTest, TestGetIndices) {
    int count;
    Vector3<int> num_gridpt_1D = {6, 3, 4};
    double h = 1.0;
    Vector3<int> bottom_corner  = {0, 0, 0};
    Grid grid = Grid(num_gridpt_1D, h, bottom_corner);

    // Check expand 1D index
    count = 0;
    for (const auto& [index_flat, index_3d] : grid.get_indices()) {
        EXPECT_EQ(count++, index_flat);
        EXPECT_TRUE(CompareMatrices(index_3d,
                                    grid.Expand1DIndex(index_flat)));
    }
}

GTEST_TEST(GridClassTest, TestResetStatesAndAccumulationAndRescale) {
    Vector3<int> num_gridpt_1D = {6, 3, 4};
    double h = 1.0;
    Vector3<int> bottom_corner  = {0, 0, 0};
    Grid grid = Grid(num_gridpt_1D, h, bottom_corner);
    double tmpscaling = 1.0;
    // Test the geometry of the grid
    EXPECT_EQ(grid.get_num_gridpt(), 72);
    EXPECT_TRUE(CompareMatrices(grid.get_num_gridpt_1D(),
                                Vector3<int>(6, 3, 4)));
    EXPECT_EQ(grid.get_h(), 1.0);
    EXPECT_TRUE(CompareMatrices(grid.get_bottom_corner(), bottom_corner));

    for (int k = 0; k < 4; ++k) {
    for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 6; ++i) {
        // Randomly put some values in
        tmpscaling = 1.2*k + 0.3*j + i;
        grid.set_mass(i, j, k, tmpscaling);
        grid.set_velocity(i, j, k, Vector3<double>(tmpscaling,
                                                  -tmpscaling,
                                                   tmpscaling));
        grid.set_force(i, j, k, Vector3<double>(-tmpscaling,
                                                 tmpscaling,
                                                -tmpscaling));
    }
    }
    }

    for (int k = 1; k < 4; ++k) {
    for (int j = 1; j < 3; ++j) {
    for (int i = 1; i < 6; ++i) {
        EXPECT_TRUE(!CompareMatrices(grid.get_velocity(i, j, k),
                                     Vector3<double>::Zero()));
    }
    }
    }

    grid.ResetStates();

    // Test ResetStates sets all states to zero
    for (int k = 0; k < 4; ++k) {
    for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 6; ++i) {
        EXPECT_TRUE(CompareMatrices(grid.get_velocity(i, j, k),
                                    Vector3<double>::Zero()));
        EXPECT_TRUE(CompareMatrices(grid.get_force(i, j, k),
                                    Vector3<double>::Zero()));
        EXPECT_EQ(grid.get_mass(i, j, k), 0.0);
    }
    }
    }

    // Test accumulation
    Vector3<int> grid_index;
    for (int k = 0; k < 4; ++k) {
    for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 6; ++i) {
        grid_index(0) = i;
        grid_index(1) = j;
        grid_index(2) = k;
        // Randomly put some values in
        grid.AccumulateMass(i, j, k, 1.0);
        grid.AccumulateVelocity(i, j, k, Vector3<double>(1.0, -1.0, 1.0));
        grid.AccumulateForce(i, j, k, Vector3<double>(-1.0, -1.0, 1.0));
        EXPECT_TRUE(CompareMatrices(grid.get_velocity(i, j, k),
                                    Vector3<double>(1.0, -1.0, 1.0), kEps));
        EXPECT_TRUE(CompareMatrices(grid.get_force(i, j, k),
                                    Vector3<double>(-1.0, -1.0, 1.0), kEps));
        EXPECT_EQ(grid.get_mass(i, j, k), 1.0);

        grid.AccumulateMass(grid_index, 1.2);
        grid.AccumulateVelocity(grid_index, Vector3<double>(1.2, -1.2, 1.2));
        grid.AccumulateForce(grid_index, Vector3<double>(-1.2, -1.2, 1.2));
        EXPECT_TRUE(CompareMatrices(grid.get_velocity(i, j, k),
                                    Vector3<double>(2.2, -2.2, 2.2), kEps));
        EXPECT_TRUE(CompareMatrices(grid.get_force(i, j, k),
                                    Vector3<double>(-2.2, -2.2, 2.2), kEps));
        EXPECT_EQ(grid.get_mass(i, j, k), 2.2);
    }
    }
    }

    // Test rescale
    grid.RescaleVelocities();
    for (int k = 0; k < 4; ++k) {
    for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 6; ++i) {
        // Randomly put some values in
        EXPECT_TRUE(CompareMatrices(grid.get_velocity(i, j, k),
                                    Vector3<double>(1.0, -1.0, 1.0), kEps));
        EXPECT_TRUE(CompareMatrices(grid.get_force(i, j, k),
                                    Vector3<double>(-2.2, -2.2, 2.2), kEps));
        EXPECT_EQ(grid.get_mass(i, j, k), 2.2);
    }
    }
    }
}

GTEST_TEST(GridClassTest, TestUpdateVelocity) {
    Vector3<int> num_gridpt_1D = {6, 3, 4};
    double h = 1.0;
    Vector3<int> bottom_corner  = {0, 0, 0};
    Grid grid = Grid(num_gridpt_1D, h, bottom_corner);
    double tmpscaling = 1.0;
    double dt = 0.2;

    for (int k = 0; k < 4; ++k) {
    for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 6; ++i) {
        // Randomly put some values in
        tmpscaling = 1.2*k + 0.3*j + i;
        grid.set_mass(i, j, k, tmpscaling);
        grid.set_velocity(i, j, k, Vector3<double>(tmpscaling,
                                                  -tmpscaling,
                                                   tmpscaling));
        grid.set_force(i, j, k, Vector3<double>(-tmpscaling,
                                                 tmpscaling,
                                                -tmpscaling));
    }
    }
    }

    grid.UpdateVelocity(dt);

    // v^{n+1} = v^n + dt*f^n/m = [t, -t, t] + dt*[-t, t, -t]/t
    //                          = [t, -t, t] + [-dt, dt, -dt]
    //                          = [t-dt, -t+dt, t-dt]
    // t: tmpscaling
    for (int k = 1; k < 4; ++k) {
    for (int j = 1; j < 3; ++j) {
    for (int i = 1; i < 6; ++i) {
        tmpscaling = 1.2*k + 0.3*j + i;
        EXPECT_TRUE(CompareMatrices(grid.get_velocity(i, j, k),
            Vector3<double>(tmpscaling-dt, -tmpscaling+dt, tmpscaling-dt),
            kEps));
    }
    }
    }
}

GTEST_TEST(GridClassTest, TestWallBoundaryCondition1) {
    // In this test case, we enforce slip wall boundary condition to the
    // boundary of a 10x20x30 grid.
    Vector3<int> num_gridpt_1D = {10, 20, 30};
    double h = 1.0;
    Vector3<int> bottom_corner  = {0, 0, 0};
    Grid grid = Grid(num_gridpt_1D, h, bottom_corner);
    // We assume an ideal slip boundary condition
    double mu = 0.0;
    BoundaryCondition bc = BoundaryCondition();

    // Initialize the boundary spaces
    bc.AddBoundary({mu, {{-1, 0, 0}, {9, 0, 0}}});
    bc.AddBoundary({mu, {{1, 0, 0}, {0, 0, 0}}});
    bc.AddBoundary({mu, {{0, -1, 0}, {0, 19, 0}}});
    bc.AddBoundary({mu, {{0, 1, 0}, {0, 0, 0}}});
    bc.AddBoundary({mu, {{0, 0, -1}, {0, 0, 29}}});
    bc.AddBoundary({mu, {{0, 0, 1}, {0, 0, 0}}});

    // Populate the grid with nonzero velocities
    for (int k = bottom_corner(2); k < bottom_corner(2)+num_gridpt_1D(2); ++k) {
    for (int j = bottom_corner(1); j < bottom_corner(1)+num_gridpt_1D(1); ++j) {
    for (int i = bottom_corner(0); i < bottom_corner(0)+num_gridpt_1D(0); ++i) {
        grid.set_velocity(i, j, k, Vector3<double>(1.0, 1.0, 1.0));
        EXPECT_TRUE(!grid.get_velocity(i, j, k).isZero());
    }
    }
    }

    // Enforce slip BC
    grid.EnforceBoundaryCondition(bc);

    // Check velocity after enforcement, hardcode values for verification
    for (int k = bottom_corner(2); k < bottom_corner(2)+num_gridpt_1D(2); ++k) {
    for (int j = bottom_corner(1); j < bottom_corner(1)+num_gridpt_1D(1); ++j) {
    for (int i = bottom_corner(0); i < bottom_corner(0)+num_gridpt_1D(0); ++i) {
        const Vector3<double>& velocity_i = grid.get_velocity(i, j, k);
        if (i == bottom_corner(0)
         || i == bottom_corner(0)+num_gridpt_1D(0)-1) {
            EXPECT_EQ(velocity_i(0), 0);
        } else {
            EXPECT_EQ(velocity_i(0), 1);
        }
        if (j == bottom_corner(1)
         || j == bottom_corner(1)+num_gridpt_1D(1)-1) {
            EXPECT_EQ(velocity_i(1), 0);
        } else {
            EXPECT_EQ(velocity_i(1), 1);
        }
        if (k == bottom_corner(2)
         || k == bottom_corner(2)+num_gridpt_1D(2)-1) {
            EXPECT_EQ(velocity_i(2), 0);
        } else {
            EXPECT_EQ(velocity_i(2), 1);
        }
    }
    }
    }
}

GTEST_TEST(GridClassTest, TestWallBoundaryCondition2) {
    // In this test case, we enforce frictional wall boundary condition to the
    // plane on a 21x21x21 grid of the domain [0, 10]x[0, 10]x[0, 10]. The
    // boundary plane has outward normal (1, 1, 1), contains the point (5, 5,
    // 5).
    Vector3<int> num_gridpt_1D = {21, 21, 21};
    double h = 0.5;
    Vector3<int> bottom_corner  = {0, 0, 0};
    Vector3<double> velocity_grid = Vector3<double>(1.0, 2.0, 3.0);
    Grid grid = Grid(num_gridpt_1D, h, bottom_corner);
    // Friction coefficient
    double mu = 0.05;
    BoundaryCondition bc = BoundaryCondition();

    // Initialize the boundary spaces
    bc.AddBoundary({mu, {{-1, -1, -1}, {5, 5, 5}}});

    // Populate the grid with nonzero velocities
    for (int k = bottom_corner(2); k < bottom_corner(2)+num_gridpt_1D(2); ++k) {
    for (int j = bottom_corner(1); j < bottom_corner(1)+num_gridpt_1D(1); ++j) {
    for (int i = bottom_corner(0); i < bottom_corner(0)+num_gridpt_1D(0); ++i) {
        grid.set_velocity(i, j, k, velocity_grid);
        EXPECT_TRUE(!grid.get_velocity(i, j, k).isZero());
    }
    }
    }

    // Enforce wall boundary condition
    grid.EnforceBoundaryCondition(bc);

    // Check velocity after enforcement, hardcode values for verification
    for (int k = bottom_corner(2); k < bottom_corner(2)+num_gridpt_1D(2); ++k) {
    for (int j = bottom_corner(1); j < bottom_corner(1)+num_gridpt_1D(1); ++j) {
    for (int i = bottom_corner(0); i < bottom_corner(0)+num_gridpt_1D(0); ++i) {
        const Vector3<double>& position_i = grid.get_position(i, j, k);
        const Vector3<double>& velocity_i = grid.get_velocity(i, j, k);
        double dist =
            bc.get_boundary(0).boundary_space.CalcSignedDistance(position_i);
        // If the grid point is not on/in the boundary, velocity shall be the
        // same
        if (dist > 0) {
            EXPECT_TRUE(CompareMatrices(velocity_i,
                                        Vector3<double>{1.0, 2.0, 3.0}, kEps));
        } else {
            // Given v_i = (1, 2, 3), n = 1/sqrt(3)*(-1, -1, -1)
            // Then v_t = (-1, 0, 1), vn = 6/sqrt(3)
            // v = vt - \mu v_n vt/\|vt\| = (1-0.05\sqrt(6)) (-1, 0, 1)
            EXPECT_TRUE(CompareMatrices(velocity_i,
                            (1.0-0.05*sqrt(6))*Vector3<double>(-1.0, 0.0, 1.0),
                            kEps));
        }
    }
    }
    }
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
