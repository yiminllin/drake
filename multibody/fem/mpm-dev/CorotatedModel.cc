#include "drake/multibody/fem/mpm-dev/CorotatedModel.h"

namespace drake {
namespace multibody {
namespace mpm {

CorotatedModel::CorotatedModel(): ConstitutiveModel() {}

CorotatedModel::CorotatedModel(double E, double nu): ConstitutiveModel(E, nu) {}

void CorotatedModel::CalcFirstPiolaKirchhoffStress(
        const Matrix3<double>& F, Matrix3<double>* P) const {
    Matrix3<double> R, S, JFinvT;
    double J = F.determinant();
    fem::internal::PolarDecompose<double>(F, &R, &S);
    fem::internal::CalcCofactorMatrix<double>(F, &JFinvT);
    *P = 2.0*mu_*(F-R) + lambda_*(J-1.0)*JFinvT;
}

void CorotatedModel::CalcKirchhoffStress(const Matrix3<double>& F,
                                         Matrix3<double>* tau) const {
    Matrix3<double> R, S;
    double J = F.determinant();
    fem::internal::PolarDecompose<double>(F, &R, &S);
    *tau = 2.0*mu_*(F-R)*F.transpose()
         + lambda_*(J-1.0)*J*Matrix3<double>::Identity();
}

void CorotatedModel::CalcFirstPiolaKirchhoffStressAndKirchhoffStress(
        const Matrix3<double>& F, Matrix3<double>* P,
        Matrix3<double>* tau) const {
    Matrix3<double> R, S, JFinvT;
    double J = F.determinant();
    fem::internal::PolarDecompose<double>(F, &R, &S);
    fem::internal::CalcCofactorMatrix<double>(F, &JFinvT);
    *P = 2.0*mu_*(F-R) + lambda_*(J-1.0)*JFinvT;
    *tau = (*P)*F.transpose();
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake


