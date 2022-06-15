#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/matrix_utilities.h"

namespace drake {
namespace multibody {
namespace mpm {

// A implementation of Fixed Corotated Model (Constitutive Model)
class CorotatedModel {
 public:
    // Default material, dough: E = 9e4 Pa, nu = 0.49
    CorotatedModel();

    // Constructor uses Young's modulus E and Poisson's ratio nu
    CorotatedModel(double E, double nu);

    // First Piola Kirchhoff stress density: P = dpsi/dF
    void CalcFirstPiolaKirchhoffStress(
        const Matrix3<double>& F, EigenPtr<Matrix3<double>> P) const;

    // Kirchhoff stress density: tau = P F^T = dpsi/dF F^T
    void CalcKirchhoffStress(const Matrix3<double>& F,
                             EigenPtr<Matrix3<double>> tau) const;
    void CalcFirstPiolaKirchhoffStressAndKirchhoffStress(
        const Matrix3<double>& F, EigenPtr<Matrix3<double>> P,
        EigenPtr<Matrix3<double>> tau) const;

 private:
    double mu_;                         // Parameters in defining
    double lambda_;                     // the energy density function
};  // class CorotatedModel

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
