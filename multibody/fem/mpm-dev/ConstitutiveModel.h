#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/matrix_utilities.h"

namespace drake {
namespace multibody {
namespace mpm {

// A base class providing the interface of Constitutive model
class ConstitutiveModel {
 public:
    // Default material, dough: E = 9e4 Pa, nu = 0.49
    ConstitutiveModel();

    // Constructor uses Young's modulus E and Poisson's ratio nu
    ConstitutiveModel(double E, double nu);

    virtual std::unique_ptr<ConstitutiveModel> Clone() const = 0;

    // First Piola Kirchhoff stress density: P = dpsi/dF
    virtual void CalcFirstPiolaKirchhoffStress(
            const Matrix3<double>& F, Matrix3<double>* P) const = 0;

    // Kirchhoff stress density: tau = P F^T = dpsi/dF F^T
    virtual void CalcKirchhoffStress(const Matrix3<double>& F,
                                     Matrix3<double>* tau) const = 0;
    virtual void CalcFirstPiolaKirchhoffStressAndKirchhoffStress(
                        const Matrix3<double>& F, Matrix3<double>* P,
                        Matrix3<double>* tau) const = 0;

    virtual ~ConstitutiveModel() = default;

 protected:
    double mu_;                         // Parameters in defining
    double lambda_;                     // the energy density function
};  // class ConstitutiveModel

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
