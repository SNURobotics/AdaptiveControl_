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

MatrixXd Mapping_Inertia::TensorVec2RiemMetric(VectorXd phi)
{
	MatrixXd P = TensorMat2MomentMat(TensorVec2TensorMat(phi));
	MatrixXd P_inv = P.inverse();
	MatrixXd TensorVec_RiemMetric = MatrixXd::Zero(10,10);
	VectorXd temp_i; MatrixXd temp_P_i;
	VectorXd temp_j; MatrixXd temp_P_j;
	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			temp_i = VectorXd::Zero(10); temp_i[i] = 1.0;
			temp_j = VectorXd::Zero(10); temp_j[j] = 1.0;
			temp_P_i = TensorMat2MomentMat(TensorVec2TensorMat(temp_i));
			temp_P_j = TensorMat2MomentMat(TensorVec2TensorMat(temp_j));

			TensorVec_RiemMetric(i, j) = (temp_P_i * P_inv * temp_P_j * P_inv).trace();
		}
	}
	return TensorVec_RiemMetric;
}
MatrixXd Mapping_Inertia::coTensorVec2coMomentMat(VectorXd co_phi)
{
	MatrixXd coMomentMat = MatrixXd::Zero(4, 4);
	coMomentMat(0, 0) = co_phi[1] + co_phi[2];
	coMomentMat(1, 1) = co_phi[2] + co_phi[0];
	coMomentMat(2, 2) = co_phi[0] + co_phi[1];
	
	coMomentMat(0, 1) = -co_phi[3] / 2; coMomentMat(1, 0) = -co_phi[3] / 2;
	coMomentMat(0, 2) = -co_phi[4] / 2; coMomentMat(2, 0) = -co_phi[4] / 2;
	coMomentMat(1, 2) = -co_phi[5] / 2; coMomentMat(2, 1) = -co_phi[5] / 2;

	coMomentMat(0, 3) = co_phi[6] / 2; coMomentMat(3, 0) = co_phi[6] / 2;
	coMomentMat(1, 3) = co_phi[7] / 2; coMomentMat(3, 1) = co_phi[6] / 2;
	coMomentMat(2, 3) = co_phi[8] / 2; coMomentMat(3, 2) = co_phi[6] / 2;

	coMomentMat(3, 3) = co_phi[9];

	return coMomentMat;
}

Inertia* Mapping_Inertia::TensorVec2Inertia(VectorXd phi)
{
	Inertia* I = new Inertia;
	I->_I[0] = (SR_real)phi[0];
	I->_I[1] = (SR_real)phi[1];
	I->_I[2] = (SR_real)phi[2];
	I->_I[3] = (SR_real)phi[3];
	I->_I[4] = (SR_real)phi[4];
	I->_I[5] = (SR_real)phi[5];
	I->_I[6] = (SR_real)phi[6];
	I->_I[7] = (SR_real)phi[7];
	I->_I[8] = (SR_real)phi[8];
	I->_I[9] = (SR_real)phi[9];
	return I;
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

vector<Inertia*> Mapping_Inertia::Set_Initial_Inertia_estimate(vector<Inertia*> I_)
{
	vector<Inertia*> I;
	I = I_;
	vector<MatrixXd> G(I.size());
	vector<MatrixXd> P(I.size());
	for (int i = 0; i < I.size(); i++)
	{
		VectorXd tmp_p; double tmp_m;
		G[i] = MatrixXd::Zero(6, 6);
		P[i] = MatrixXd::Zero(4,4);
		I_[i]->ToArray(G[i].data());
		P[i] = TensorMat2MomentMat(G[i]);

		tmp_m = P[i](3, 3);
		tmp_p = P[i].block<3, 1>(0, 3) / tmp_m;

		P[i].block<3, 3>(0, 0) = tmp_m * tmp_p*tmp_p.transpose() + tmp_m*0.001 * MatrixXd::Identity(4,4);
		//cout << P[i] << endl;
		I[i] = TensorVec2Inertia(TensorMat2TensorVec(MomentMat2TensorMat(P[i])));
	}

	return I;
}