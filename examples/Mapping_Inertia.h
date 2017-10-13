#pragma once

#include "robot1.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "LieGroup/LieGroup.h"
#include <vector>
#include <math.h>

using namespace Eigen;

namespace Mapping_Inertia
{
	MatrixXd TensorMat2MomentMat(MatrixXd G);
	MatrixXd MomentMat2TensorMat(MatrixXd P);
	VectorXd TensorMat2TensorVec(MatrixXd G);
	MatrixXd TensorVec2TensorMat(VectorXd phi);
	Inertia* TensorVec2Inertia(VectorXd phi);

	MatrixXd TensorVec2RiemMetric(VectorXd phi);
	MatrixXd coTensorVec2coMomentMat(VectorXd co_phi);

	pair<SE3, Vector3d> principal_decomp_tensor(Inertia I);

	vector<Inertia*> Set_Initial_Inertia_estimate(vector<Inertia*> I_);
}