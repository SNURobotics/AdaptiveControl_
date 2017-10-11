#include "Mapping_Inertia.h"

using namespace Eigen;

//Matrix<double, 6, 6> TensorMat2MomentMat(Matrix<double, 4, 4> P)
//{
//
//}
//Matrix<double, 4, 4> MomentMat2TensorMat(Matrix<double, 6, 6> P)
//{
//
//}
//Matrix<double, 10, 1> TensorMat2TensorVec(Matrix<double, 4, 4> P)
//{
//
//}
//Matrix<double, 6, 6> TensorVec2TensorMat(Matrix<double, 10, 1> P)
//{
//
//}

std::pair<SE3, Vector3d> Mapping_Inertia::principal_decomp_tensor(Inertia I)
{
	Vector3d sigma;
	Vector3d sigma_;

	Vec3 offset = I.GetOffset();
	SR_real mass = I.GetMass();

	Vec3 com = offset / mass;

	Eigen::Matrix<double, 6, 6> I_tensor;

	I.ToArray(I_tensor.data());
	
	Matrix3d I_b = I_tensor.block<3, 3>(0, 0);
	Matrix3d h_b_mat = I_tensor.block<3, 3>(0, 3);

	Matrix3d I_com = I_b - (h_b_mat*h_b_mat.transpose())/((double)mass);

	SelfAdjointEigenSolver<Matrix3d> es(I_com);
	sigma = es.eigenvalues();
	Matrix3d R = es.eigenvectors();
	if (R.determinant() < 0)
	{
		Vector3d v2 = R.block<3, 1>(0, 1);
		Vector3d v3 = R.block<3, 1>(0, 2);
		R.block<3, 1>(0, 1) = v3;
		R.block<3, 1>(0, 2) = v2;

		double sigma2 = sigma(1);
		double sigma3 = sigma(2);
		sigma(1) = sigma3;
		sigma(2) = sigma2;
	}

	SE3 T((SR_real)R(0,0), (SR_real)R(1, 0), (SR_real)R(2, 0), (SR_real)R(0, 1), (SR_real)R(1, 1), (SR_real)R(2, 1), (SR_real)R(0, 2), (SR_real)R(1, 2), (SR_real)R(2, 2), com[0], com[1], com[2]);

	sigma_(0) = sqrt((5.0 /2.0)*(-sigma(0)+sigma(1)+sigma(2))/mass);
	sigma_(1) = sqrt((5.0 / 2.0)*(sigma(0) - sigma(1) + sigma(2)) / mass);
	sigma_(2) = sqrt((5.0 / 2.0)*(sigma(0) + sigma(1) - sigma(2)) / mass);

	return std::make_pair(T, sigma_);
}
