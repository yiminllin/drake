#include "drake/multibody/fem/mpm-dev/BoundaryCondition.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace mpm {

constexpr double TOLERANCE = 1e-10;

namespace {

GTEST_TEST(BoundaryConditionTest, TestDefaultConstructor) {
    BoundaryCondition bc = BoundaryCondition();
    EXPECT_EQ(bc.get_num_boundaries(), 0);
}

GTEST_TEST(BoundaryConditionTest, TestGetAdd) {
    // Construct a boundary condition class with two boundaries
    double mu0 = 0.1;
    double mu1 = 0.2;
    double mu2 = 0.3;
    Vector3<double> n0 = Vector3<double>(1.0, 0.0, 0.0);
    Vector3<double> p0 = Vector3<double>(2.0, 2.0, 2.0);
    Vector3<double> n1 = Vector3<double>(0.0, 1.0, 0.0);
    Vector3<double> p1 = Vector3<double>(3.0, 3.0, 3.0);
    Vector3<double> n2 = Vector3<double>(0.0, 0.0, 1.0);
    Vector3<double> p2 = Vector3<double>(1.0, 1.0, 1.0);
    BoundaryCondition::Boundary b0 = {mu0, {n0, p0}};
    BoundaryCondition::Boundary b1 = {mu1, {n1, p1}};
    BoundaryCondition::Boundary b2 = {mu2, {n2, p2}};
    std::vector<BoundaryCondition::Boundary> boundaries = {b0, b1};
    BoundaryCondition bc = BoundaryCondition(boundaries);

    EXPECT_EQ(bc.get_num_boundaries(), 2);
    const std::vector<BoundaryCondition::Boundary>& boundaries_ret
                                                        = bc.get_boundaries();
    EXPECT_EQ(boundaries_ret[0].friction_coefficient, mu0);
    EXPECT_EQ(bc.get_boundary(0).friction_coefficient, mu0);
    EXPECT_EQ(boundaries_ret[1].friction_coefficient, mu1);
    EXPECT_EQ(bc.get_boundary(1).friction_coefficient, mu1);
    EXPECT_TRUE(CompareMatrices(boundaries_ret[0].boundary_space.normal(), n0));
    EXPECT_TRUE(CompareMatrices(bc.get_boundary(0).boundary_space.normal(),
                                                                           n0));
    EXPECT_TRUE(CompareMatrices(boundaries_ret[1].boundary_space.normal(), n1));
    EXPECT_TRUE(CompareMatrices(bc.get_boundary(1).boundary_space.normal(),
                                                                           n1));

    // Add a boundary space to BoundaryCondition
    bc.AddBoundary(b2);
    EXPECT_EQ(bc.get_num_boundaries(), 3);
    EXPECT_EQ(boundaries_ret[0].friction_coefficient, mu0);
    EXPECT_EQ(bc.get_boundary(0).friction_coefficient, mu0);
    EXPECT_EQ(boundaries_ret[1].friction_coefficient, mu1);
    EXPECT_EQ(bc.get_boundary(1).friction_coefficient, mu1);
    EXPECT_EQ(boundaries_ret[2].friction_coefficient, mu2);
    EXPECT_EQ(bc.get_boundary(2).friction_coefficient, mu2);
    EXPECT_TRUE(CompareMatrices(boundaries_ret[0].boundary_space.normal(), n0));
    EXPECT_TRUE(CompareMatrices(bc.get_boundary(0).boundary_space.normal(),
                                                                           n0));
    EXPECT_TRUE(CompareMatrices(boundaries_ret[1].boundary_space.normal(), n1));
    EXPECT_TRUE(CompareMatrices(bc.get_boundary(1).boundary_space.normal(),
                                                                           n1));
    EXPECT_TRUE(CompareMatrices(boundaries_ret[2].boundary_space.normal(), n2));
    EXPECT_TRUE(CompareMatrices(bc.get_boundary(2).boundary_space.normal(),
                                                                           n2));
}

GTEST_TEST(BoundaryConditionTest, TestApply) {
    // Construct a boundary
    double mu0 = 0.1;
    Vector3<double> n0 = Vector3<double>(1.0, 1.0, 1.0);
    Vector3<double> p0 = Vector3<double>(2.0, 2.0, 2.0);
    BoundaryCondition::Boundary b0 = {mu0, {n0, p0}};
    std::vector<BoundaryCondition::Boundary> boundaries = {b0};
    BoundaryCondition bc = BoundaryCondition(boundaries);

    // First consider a point inside the domain, no boundary condition shall be
    // enforced.
    Vector3<double> pos0 = {3.0, 3.0, 3.0};
    Vector3<double> vel0 = {1.0, -1.0, 0.8};
    bc.Apply(pos0, &vel0);
    ASSERT_TRUE(CompareMatrices(vel0, Vector3<double>{1.0, -1.0, 0.8},
                                                                    TOLERANCE));

    // Next consider a point on the boundary half plane, boundary condition
    // shall be enforced, the velocity is in the normal direction, so the new
    // velocity should be zero.
    Vector3<double> pos1 = {2.0, 2.0, 2.0};
    Vector3<double> vel1 = {10.0, 10.0, 10.0};
    bc.Apply(pos1, &vel1);
    ASSERT_TRUE(CompareMatrices(vel1, Vector3<double>::Zero(), TOLERANCE));

    // Next consider a point on the boundary half plane. The velocity is in not
    // the normal direction, but the normal impulse and the consequent friction
    // is so large that only the tangential velocity is left after hitting the
    // boundary.
    Vector3<double> pos2 = {2.0, 2.0, 2.0};
    Vector3<double> vel2 = {11.0, 10.0, 10.0};
    bc.Apply(pos2, &vel2);
    ASSERT_TRUE(CompareMatrices(vel2, Vector3<double>::Zero(), TOLERANCE));

    // Finally test the frictional wall boundary condition enforcement using
    // a different velocity such that the friction will not be too large
    Vector3<double> pos3 = {2.0, 2.0, 2.0};
    Vector3<double> vel3 = {2.0, 1.0, 1.0};
    bc.Apply(pos3, &vel3);
    ASSERT_TRUE(CompareMatrices(vel3, (1-mu0*4.0/sqrt(2))/3.0
                                *Vector3<double>{2.0, -1.0, -1.0}, TOLERANCE));
}

}  // namespace
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
