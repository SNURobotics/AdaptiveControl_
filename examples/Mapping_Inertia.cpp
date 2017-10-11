#include "Mapping_Inertia.h"

using namespace Eigen;

MatrixXd Mapping_Inertia::TensorMat2MomentMat(MatrixXd G)
{
	Matrix<double, 4, 4> P;
	Vector3d h;
	h << -G(1, 5), G(0, 5), -G(0, 4);
	P.block<3, 3>(0, 0) = 0.5 * G.block<3, 3>(0, 0).trace() * MatrixXd::Identity(3, 3) - G.block<3, 3>(0, 0);
	P.block<3, 1>(0, 3) = h;
	P.block<1, 3>(3, 0) = h.transpose();
	P(3, 3) = G(3, 3);

	return P;
}
MatrixXd Mapping_Inertia::MomentMat2TensorMat(MatrixXd P)
{
	MatrixXd G = MatrixXd::Zero(6, 6);
	Vector3d h = P.block<3, 1>(0, 3);
	G.block<3, 3>(0, 0) = P.block<3, 3>(0, 0).trace() * MatrixXd::Identity(3, 3) - P.block<3, 3>(0, 0);
	G(1, 5) = -h(0); G(0, 5) = h(1); G(0, 4) = -h(2);
	G(2, 4) = h(0); G(2, 3) = -h(1); G(1, 3) = h(2);
	G.block<3, 3>(3, 0) = G.block<3, 3>(0, 3).transpose();
	G(3, 3) = P(3, 3);
	G(4, 4) = P(3, 3);
	G(5, 5) = P(3, 3);

	return G;
}
VectorXd Mapping_Inertia::TensorMat2TensorVec(MatrixXd G)
{
	VectorXd phi(10);

	phi << G(0, 0), G(1, 1), G(2, 2), G(0, 1), G(0, 2), G(1, 2), -G(1, 5), G(0, 5), -G(0, 4), G(3, 3);
	return phi;
}
MatrixXd Mapping_Inertia::TensorVec2TensorMat(VectorXd phi)
{
	Inertia I((SR_real)phi(0), (SR_real)phi(1), (SR_real)phi(2), (SR_real)phi(3), (SR_real)phi(4), (SR_real)phi(5), (SR_real)phi(6), (SR_real)phi(7), (SR_real)phi(8), (SR_real)phi(9));
	Matrix<double, 6, 6> G;
	I.ToArray(G.data());
	return G;
}

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

	Matrix3d I_com = I_b - (h_b_mat*h_b_mat.transpose()) / ((double)mass);

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

	SE3 T((SR_real)R(0, 0), (SR_real)R(1, 0), (SR_real)R(2, 0), (SR_real)R(0, 1), (SR_real)R(1, 1), (SR_real)R(2, 1), (SR_real)R(0, 2), (SR_real)R(1, 2), (SR_real)R(2, 2), com[0], com[1], com[2]);

	sigma_(0) = sqrt((5.0 / 2.0)*(-sigma(0) + sigma(1) + sigma(2)) / mass);
	sigma_(1) = sqrt((5.0 / 2.0)*(sigma(0) - sigma(1) + sigma(2)) / mass);
	sigma_(2) = sqrt((5.0 / 2.0)*(sigma(0) + sigma(1) - sigma(2)) / mass);

	return std::make_pair(T, sigma_);
}
