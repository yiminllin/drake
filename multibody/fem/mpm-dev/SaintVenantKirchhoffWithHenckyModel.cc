#include "drake/multibody/fem/mpm-dev/SaintVenantKirchhoffWithHenckyModel.h"

namespace drake {
namespace multibody {
namespace mpm {

SaintVenantKirchhoffWithHenckyModel::SaintVenantKirchhoffWithHenckyModel():
                                                        ConstitutiveModel() {}

SaintVenantKirchhoffWithHenckyModel::SaintVenantKirchhoffWithHenckyModel(
                                                        double E, double nu):
                                                    ConstitutiveModel(E, nu) {}

void SaintVenantKirchhoffWithHenckyModel::CalcFirstPiolaKirchhoffStress(
        const Matrix3<double>& F, Matrix3<double>* P) const {
    Eigen::JacobiSVD<Matrix3<double>>
                            svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Matrix3<double>& U  = svd.matrixU();
    const Matrix3<double>& V  = svd.matrixV();
    Vector3<double> sigma     = svd.singularValues();
    // logs of singular values
    Vector3<double> log_sigma = sigma.array().log();
    // sum of logs of singular values of F: ∑ ln(σᵢ)
    double sum_log_sigma      = log_sigma.sum();
    // Overwrite the the vector sigma [σᵢ] with the singular values of the first
    // Piola Kirchhoff stress:
    // Σᵢ = 1/σᵢ ⋅ [2μ log(σᵢ) + λ ∑ᵢ log(σᵢ)]
    for (int d = 0; d < 3; ++d) {
        sigma(d) = (2*mu_*log_sigma(d) + lambda_*sum_log_sigma)/sigma(d);
    }
    // The first Piola Kirchhoff stress can be then written as
    // P = U Σᵢ Vᵀ, where U, V are left and right singular vectors of the
    //             deformation gradient F
    *P = U * sigma.asDiagonal() * V.transpose();
}

void SaintVenantKirchhoffWithHenckyModel::CalcKirchhoffStress
                                        (const Matrix3<double>& F,
                                         Matrix3<double>* tau) const {
    Eigen::JacobiSVD<Matrix3<double>>
                            svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Matrix3<double>& U = svd.matrixU();
    Vector3<double> sigma    = svd.singularValues();
    // logs of singular values
    Vector3<double> log_sigma = sigma.array().log();
    // sum of logs of singular values of F: ∑ ln(σᵢ)
    double sum_log_sigma      = log_sigma.sum();
    // Overwrite the the vector sigma [σᵢ] with the singular values of the
    // Kirchhoff stress:
    // Σᵢ = 2μ log(σᵢ) + λ ∑ᵢ log(σᵢ)
    for (int d = 0; d < 3; ++d) {
        sigma(d)  = 2*mu_*log_sigma(d) + lambda_*sum_log_sigma;
    }
    // The Kirchhoff stress can be then written as
    // τ = U Σᵢ Uᵀ, where U are left singular vectors of the deformation
    //             gradient F
    *tau = U * sigma.asDiagonal() * U.transpose();
}

void SaintVenantKirchhoffWithHenckyModel::
                                CalcFirstPiolaKirchhoffStressAndKirchhoffStress(
        const Matrix3<double>& F, Matrix3<double>* P,
        Matrix3<double>* tau) const {
    Eigen::JacobiSVD<Matrix3<double>>
                            svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Matrix3<double>& U = svd.matrixU();
    const Matrix3<double>& V = svd.matrixV();
    Vector3<double> sigma    = svd.singularValues();
    // logs of singular values
    Vector3<double> log_sigma = sigma.array().log();
    // sum of logs of singular values of F: ∑ ln(σᵢ)
    double sum_log_sigma      = log_sigma.sum();
    // Overwrite the the vector sigma [σᵢ] with the singular values of the first
    // Piola Kirchhoff stress:
    // Σᵢ = 1/σᵢ ⋅ [2μ log(σᵢ) + λ ∑ᵢ log(σᵢ)]
    for (int d = 0; d < 3; ++d) {
        sigma(d) = (2*mu_*log_sigma(d) + lambda_*sum_log_sigma)/sigma(d);
    }
    // The first Piola Kirchhoff stress can be then written as
    // P = U Σᵢ Vᵀ, where U, V are left and right singular vectors of the
    //             deformation gradient F, and τ = PFᵀ
    *P   = U * sigma.asDiagonal() * V.transpose();
    *tau = (*P)*F.transpose();
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake


