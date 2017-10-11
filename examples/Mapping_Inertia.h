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
	//Matrix<double, 4, 4> TensorMat2MomentMat(Matrix<double, 6, 6> G);
	//Matrix<double, 6, 6> MomentMat2TensorMat(Matrix<double, 4, 4> P);
	//Matrix<double, 10, 1> TensorMat2TensorVec(Matrix<double, 6, 6> G);
	//Matrix<double, 6, 6> TensorVec2TensorMat(Matrix<double, 10, 1> phi);

	pair<SE3, Vector3d> principal_decomp_tensor(Inertia I);
}