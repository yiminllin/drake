#include "drake/multibody/fem/mpm-dev/ConstitutiveModel.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/fem/mpm-dev/CorotatedModel.h"
#include "drake/multibody/fem/mpm-dev/SaintVenantKirchhoffWithHenckyModel.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double TOLERANCE = 1e-10;

GTEST_TEST(CorotatedModelTest, CorotatedModelCalculationTest) {
    // Taken from fem/test/corotated_model_data_test.cc
    /* We set deformation gradient as F = R*S where R is an arbitrary rotation
    matrix and S is an arbitrary sysmmetric positive definite matrix. */
    const Matrix3<double> R =
        math::RotationMatrix<double>
            (math::RollPitchYaw<double>(pi/2.0, pi, 3.0*pi/2.0)).matrix();
    const Matrix3<double> S = (Matrix3<double>() <<
            6, 1, 2,
            1, 4, 1,
            2, 1, 5).finished();
    const Matrix3<double> P_exact1 = (Matrix3<double>() <<
            -4, -2, -8,
            10,  2,  4,
            -2, -6, -2).finished();
    const Matrix3<double> P_exact2 = (Matrix3<double>() <<
            1336,   764, -4432,
            3668,  -572, -1336,
            572,  -5004,   764).finished();
    const Matrix3<double> tau_exact1 = (Matrix3<double>() <<
             50, -42,  20,
            -42,  70, -22,
             20, -22,  28).finished();
    const Matrix3<double> tau_exact2 = (Matrix3<double>() <<
             18724, -84,  40,
            -84,  18764, -44,
             40, -44,  18680).finished();
    const Matrix3<double> F = R * S;
    Matrix3<double> F2;
    Matrix3<double> P, tau;

    // First test on E = 2.0, nu = 0.0
    CorotatedModel corotated_model = CorotatedModel(2.0, 0.0);
    corotated_model.CalcFirstPiolaKirchhoffStress(F, &P);
    corotated_model.CalcKirchhoffStress(F, &tau);
    EXPECT_TRUE(CompareMatrices(P, P_exact1, TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, tau_exact1, TOLERANCE));

    // Next test on E = 5.0, nu = 0.25
    corotated_model = CorotatedModel(5.0, 0.25);
    corotated_model.CalcFirstPiolaKirchhoffStress(F, &P);
    corotated_model.CalcKirchhoffStress(F, &tau);
    EXPECT_TRUE(CompareMatrices(P, P_exact2, TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, tau_exact2, TOLERANCE));

    // Sanity check: If F is a rotation matrix, then stresses are zero
    corotated_model = CorotatedModel(5.0, 0.25);
    F2 = math::RotationMatrix<double>
                (math::RollPitchYaw<double>(1.0, 2.0, 3.0)).matrix();
    corotated_model.CalcFirstPiolaKirchhoffStress(F2, &P);
    corotated_model.CalcKirchhoffStress(F2, &tau);
    EXPECT_TRUE(CompareMatrices(P, Matrix3<double>::Zero(), TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, Matrix3<double>::Zero(), TOLERANCE));

    corotated_model = CorotatedModel(23.4, 0.41);
    F2 = math::RotationMatrix<double>
                (math::RollPitchYaw<double>(0.1, -2.4, 13.3)).matrix();
    corotated_model.CalcFirstPiolaKirchhoffStress(F2, &P);
    corotated_model.CalcKirchhoffStress(F2, &tau);
    EXPECT_TRUE(CompareMatrices(P, Matrix3<double>::Zero(), TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, Matrix3<double>::Zero(), TOLERANCE));
}

GTEST_TEST(SaintVenantKirchhoffWithHenckyModelTest,
           SaintVenantKirchhoffWithHenckyModelCalculationTest) {
    Matrix3<double> P, tau;
    double E0      = 5.0;
    double nu0     = 0.25;
    double E1      = 23.4;
    double nu1     = 0.41;
    SaintVenantKirchhoffWithHenckyModel hencky_model0 = {E0, nu0};
    SaintVenantKirchhoffWithHenckyModel hencky_model1 = {E1, nu1};

    // Sanity check: If F is a rotation matrix, then stresses are zero
    Matrix3<double> F0 = math::RotationMatrix<double>
                (math::RollPitchYaw<double>(1.0, 2.0, 3.0)).matrix();
    hencky_model0.CalcFirstPiolaKirchhoffStress(F0, &P);
    hencky_model0.CalcKirchhoffStress(F0, &tau);
    EXPECT_TRUE(CompareMatrices(P, Matrix3<double>::Zero(), TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, Matrix3<double>::Zero(), TOLERANCE));

    Matrix3<double> F1 = math::RotationMatrix<double>
                (math::RollPitchYaw<double>(0.1, -2.4, 13.3)).matrix();
    hencky_model1.CalcFirstPiolaKirchhoffStress(F1, &P);
    hencky_model1.CalcKirchhoffStress(F1, &tau);
    EXPECT_TRUE(CompareMatrices(P, Matrix3<double>::Zero(), TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, Matrix3<double>::Zero(), TOLERANCE));

    // For another F with SVD
    //       = U ∑ V^T =  [1     0     0   [3 0 0   [ √2/2 √2/2   0
    //                     0  √2/2 -√2/2    0 2 0    -√2/2 √2/2   0
    //                     0  √2/2  √2/2]   0 0 1]       0    0   1]
    //                 =  [3/√2  3/√2     0
    //                       -1     1 -1/√2
    //                       -1     1  1/√2]
    // Then P = U ∑_P V^T, where ∑_P = 2μ ∑^-1 ln(∑) + λtr(ln(∑))∑^-1
    // Then P = [1     0     0
    //           0  √2/2 -√2/2
    //           0  √2/2  √2/2]
    //          diag[2/3μln(3)+λ/3(ln(3)+ln(2)),
    //                  μln(2)+λ/2(ln(3)+ln(2)),
    //                           λ(ln(3)+ln(2))]
    //          [ √2/2 √2/2   0
    //           -√2/2 √2/2   0
    //              0    0   1]
    // and tau = P*F^T
    Matrix3<double> F2 =
    (Matrix3<double>() << 3/sqrt(2), 3/sqrt(2),        0.0,
                                 -1,         1, -1/sqrt(2),
                                 -1,         1,  1/sqrt(2)).finished();
    Matrix3<double> U =
    (Matrix3<double>() << 1.0,       0.0,        0.0,
                          0.0, sqrt(2)/2, -sqrt(2)/2,
                          0.0, sqrt(2)/2,  sqrt(2)/2).finished();
    Matrix3<double> V =
    (Matrix3<double>() << sqrt(2)/2, -sqrt(2)/2, 0.0,
                          sqrt(2)/2,  sqrt(2)/2, 0.0,
                                0.0,        0.0, 1.0).finished();
    double sum_log_sigma = std::log(3)+std::log(2);

    // Check with the first hencky model
    double mu0     = E0/(2*(1+nu0));
    double lambda0 = E0*nu0/(1+nu0)/(1-2*nu0);
    Vector3<double> sigma0 = {2.0/3.0*mu0*std::log(3)+lambda0/3.0*sum_log_sigma,
                                      mu0*std::log(2)+lambda0/2.0*sum_log_sigma,
                                                      lambda0*sum_log_sigma};
    Matrix3<double> P0   = U*sigma0.asDiagonal()*V.transpose();
    Matrix3<double> tau0 = P0*F2.transpose();
    hencky_model0.CalcFirstPiolaKirchhoffStress(F2, &P);
    hencky_model0.CalcKirchhoffStress(F2, &tau);
    EXPECT_TRUE(CompareMatrices(P  , P0  , TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, tau0, TOLERANCE));
    hencky_model0.CalcFirstPiolaKirchhoffStressAndKirchhoffStress(F2, &P, &tau);
    EXPECT_TRUE(CompareMatrices(P  , P0  , TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, tau0, TOLERANCE));

    // Check with the next hencky model
    double mu1     = E1/(2*(1+nu1));
    double lambda1 = E1*nu1/(1+nu1)/(1-2*nu1);
    Vector3<double> sigma1 = {2.0/3.0*mu1*std::log(3)+lambda1/3.0*sum_log_sigma,
                                      mu1*std::log(2)+lambda1/2.0*sum_log_sigma,
                                                      lambda1*sum_log_sigma};
    Matrix3<double> P1   = U*sigma1.asDiagonal()*V.transpose();
    Matrix3<double> tau1 = P1*F2.transpose();
    hencky_model1.CalcFirstPiolaKirchhoffStress(F2, &P);
    hencky_model1.CalcKirchhoffStress(F2, &tau);
    EXPECT_TRUE(CompareMatrices(P  , P1  , TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, tau1, TOLERANCE));
    hencky_model1.CalcFirstPiolaKirchhoffStressAndKirchhoffStress(F2, &P, &tau);
    EXPECT_TRUE(CompareMatrices(P  , P1  , TOLERANCE));
    EXPECT_TRUE(CompareMatrices(tau, tau1, TOLERANCE));
}



}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
