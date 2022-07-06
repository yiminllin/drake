#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/matrix_utilities.h"
#include "drake/multibody/fem/mpm-dev/ConstitutiveModel.h"

namespace drake {
namespace multibody {
namespace mpm {

// A implementation of Saint-Venant Kirchhoff model, but replace the left
// Cauchy Green strain with the Hencky strain
// https://dl.acm.org/doi/abs/10.1145/2897824.2925906
// The formula of Kirchhoff stress can be found in Section 6.3
class SaintVenantKirchhoffWithHenckyModel : public ConstitutiveModel {
 public:
    SaintVenantKirchhoffWithHenckyModel();
    SaintVenantKirchhoffWithHenckyModel(double E, double nu);

    std::unique_ptr<ConstitutiveModel> Clone() const {
        return std::make_unique<SaintVenantKirchhoffWithHenckyModel>(*this);
    }

    void CalcFirstPiolaKirchhoffStress(
        const Matrix3<double>& F, Matrix3<double>* P) const final;

    void CalcKirchhoffStress(const Matrix3<double>& F,
                             Matrix3<double>* tau) const final;
    void CalcFirstPiolaKirchhoffStressAndKirchhoffStress(
        const Matrix3<double>& F, Matrix3<double>* P,
        Matrix3<double>* tau) const final;
};  // class SaintVenantKirchhoffWithHenckyModel

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
