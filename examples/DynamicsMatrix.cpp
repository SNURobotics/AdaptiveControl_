#include "DynamicsMatrix.h"

using namespace Eigen;

DynamicsMatrix::DynamicsMatrix(robot1* pRobot, AdaptiveControl* pAdaptiveControl) : mpRobot(pRobot), mpAdaptiveControl(pAdaptiveControl)
{
	mnJoint = mpRobot->R_Joint_ID::num_joints;
	mvpEstimatedInertia = mpAdaptiveControl->mvpEstimatedInertia;


	// initialize
	vdot0 = VectorXd::Zero(6); vdot0[5] = 9.8;
	qdot = VectorXd::Zero(mnJoint);
	mMatrixA = MatrixXd::Constant(6 * mnJoint, mnJoint, 0.0);
	mMatrixL = MatrixXd::Identity(6 * mnJoint, 6 * mnJoint);
	mMatrixG = MatrixXd::Constant(6 * mnJoint, 6 * mnJoint, 0.0);
	mMatrixGamma = MatrixXd::Constant(6 * mnJoint, 6 * mnJoint, 0.0);
	mMatrixCbar = MatrixXd::Constant(6 * mnJoint, 6 * mnJoint, 0.0);
	mVdot_base = MatrixXd::Constant(6 * mnJoint, 1, 0.0);
	mMatrix_adA = MatrixXd::Constant(6 * mnJoint, 6 * mnJoint, 0.0);
	mMatrix_adv_t = MatrixXd::Constant(6 * mnJoint, 6 * mnJoint, 0.0);

	M = MatrixXd::Constant(mnJoint, mnJoint, 0.0);
	C = MatrixXd::Constant(mnJoint, mnJoint, 0.0);
	N = MatrixXd::Constant(mnJoint, 1, 0.0);
	mMatrixY = MatrixXd::Constant(mnJoint, 10 * mnJoint, 0.0);

	mvCurrentSE3.resize(mnJoint + 1); // # of link = # of joint + 1. (base link)
	mvCurrentV.resize(mnJoint + 1);
	mvA.resize(mnJoint);

	// A matrix
	for (int i = 0; i < mnJoint; i++)
	{
		Vec3 JointAxis = mpRobot->m_link[i].m_Frame.GetZ(); // i-th joint axis = z-axis of (i-1)-th link frame
		MatrixXd w(3, 1); w << JointAxis[0], JointAxis[1], JointAxis[2];
		Vector3d v = CrossMatrix(mpRobot->m_link[i].m_Frame.GetPosition()) * w;
		SE3 InvT = SE3() / mpRobot->m_link[i + 1].m_Frame;

		se3 A_screw = Ad(InvT, se3(Axis(JointAxis[0], JointAxis[1], JointAxis[2]), Vec3(v[0], v[1], v[2])));
		mvA[i] = A_screw;
		MatrixXd tmp(6, 1);
		A_screw.ToArray(tmp.data());
		mMatrixA.block<6, 1>(6 * i, i) = tmp;
	}
}

void DynamicsMatrix::UpdateMatrices()
{

	// update mvCurrentSE3
	for (int i = 0; i < mnJoint + 1; i++) // # of link = # of joint + 1. (base link)
	{
		mvCurrentSE3[i] = mpRobot->m_link[i].m_Frame;
	}

	// update mvCurrentV
	for (int i = 0; i < mnJoint + 1; i++) // # of link = # of joint + 1. (base link)
	{
		mvCurrentV[i] = mpRobot->m_link[i].m_Vel; // i-th velocity w.r.t to i-th link frame. (generalized body vel.) 
	}

	// update qdot
	for (int i = 0; i < mnJoint; i++)
	{
		qdot[i] = mpRobot->m_joint[i].GetRevoluteJointState().m_rValue[1];
	}

	ComputeL();
	ComputeG();
	ComputeGamma();
	ComputeCbar();
	Compute_adA();
	Compute_adv_t(); // L, A, qdot must be computed before adv_t.
	ComputeVdot_base();

	MassMatrix();
	CoriolisMatrix();
	GravitationalVector();
	ComputeY();
}

MatrixXd DynamicsMatrix::BigAd(SE3 T)
{
	MatrixXd matBigAd(6, 6);
	matBigAd = MatrixXd::Constant(6, 6, 0.0);
	MatrixXd R(3, 3);
	T.GetOrientation().ToArray(R.data());
	Vec3 p_vec3 = T.GetPosition();

	matBigAd.block<3, 3>(0, 0) = R;
	matBigAd.block<3, 3>(3, 0) = CrossMatrix(p_vec3) * R;
	matBigAd.block<3, 3>(3, 3) = R;

	return matBigAd;
}

MatrixXd DynamicsMatrix::SmallAd(VectorXd se3vector)
{
	MatrixXd matSmallAd(6, 6);
	matSmallAd = MatrixXd::Constant(6, 6, 0.0);

	Vec3 w(se3vector[0], se3vector[1], se3vector[2]), v(se3vector[3], se3vector[4], se3vector[5]);

	matSmallAd.block<3, 3>(0, 0) = CrossMatrix(w);
	matSmallAd.block<3, 3>(3, 0) = CrossMatrix(v);
	matSmallAd.block<3, 3>(3, 3) = CrossMatrix(w);

	return matSmallAd;
}

MatrixXd DynamicsMatrix::CrossMatrix(Vec3 w)
{
	MatrixXd Cross(3, 3);
	Cross << 0.0, -w[2], w[1],
		w[2], 0.0, -w[0],
		-w[1], w[0], 0.0;
	return Cross;
}

void DynamicsMatrix::ComputeL()
{
	mMatrixL = MatrixXd::Identity(6 * mnJoint, 6 * mnJoint);
	for (int i = 1; i < mnJoint; i++)
	{
		mMatrixL.block<6, 6>(6 * i, 6 * (i - 1)) = BigAd(mvCurrentSE3[i + 1] % mvCurrentSE3[i]);
	}

	for (int i = 2; i < mnJoint; i++)
	{
		for (int j = 0; j < i - 1; j++)
		{
			mMatrixL.block<6, 6>(6 * i, 6 * j) = mMatrixL.block<6, 6>(6 * i, 6 * (i - 1)) * mMatrixL.block<6, 6>(6 * (i - 1), 6 * j);
		}
	}
}

void DynamicsMatrix::ComputeG()
{
	for (int i = 0; i < mnJoint; ++i)
	{
		MatrixXd tmp(6, 6);
		mvpEstimatedInertia[i]->ToArray(tmp.data());
		mMatrixG.block<6, 6>(6 * i, 6 * i) = tmp;
	}
}

void DynamicsMatrix::ComputeGamma()
{
	for (int i = 1; i < mnJoint; i++)
	{
		mMatrixGamma.block<6, 6>(6 * i, 6 * (i - 1)) = BigAd(mvCurrentSE3[i + 1] % mvCurrentSE3[i]);
	}
}

void DynamicsMatrix::ComputeCbar()
{
	MatrixXd tmp(6 * mnJoint, 6 * mnJoint);
	MatrixXd tmp_t(6 * mnJoint, 6 * mnJoint);
	tmp = mMatrix_adv_t * mMatrixG;
	tmp_t = tmp.transpose();
	mMatrixCbar = tmp - tmp_t;
}


void DynamicsMatrix::Compute_adA()
{
	for (int i = 0; i < mnJoint; i++)
	{
		se3 A_i = qdot[i] * mvA[i];
		VectorXd A = VectorXd::Zero(6);
		A_i.ToArray(A.data());
		mMatrix_adA.block<6, 6>(6 * i, 6 * i) = -SmallAd(A);
	}
}

void DynamicsMatrix::Compute_adv_t()
{
	MatrixXd CurrentV = mMatrixL * mMatrixA * qdot;
	for (int i = 0; i < mnJoint; i++)
	{
		MatrixXd ad_v;
		VectorXd CurrentV_i = CurrentV.block<6,1>(6*i,0);
		
		ad_v = SmallAd(CurrentV_i);
		mMatrix_adv_t.block<6, 6>(6 * i, 6 * i) = -ad_v.transpose();
	}
}

void DynamicsMatrix::ComputeVdot_base()
{
	mVdot_base.block<6, 1>(0, 0) = BigAd(mvCurrentSE3[1] % mvCurrentSE3[0]) * vdot0;
}

void DynamicsMatrix::MassMatrix()
{
	M = mMatrixA.transpose() * mMatrixL.transpose() * mMatrixG * mMatrixL * mMatrixA;
}

void DynamicsMatrix::CoriolisMatrix()
{
	C = mMatrixA.transpose() * mMatrixL.transpose() * (mMatrixG * mMatrixL * mMatrix_adA * mMatrixGamma + mMatrixCbar) * mMatrixL * mMatrixA;
}

void DynamicsMatrix::GravitationalVector()
{
	N = mMatrixA.transpose() * mMatrixL.transpose() * mMatrixG * mMatrixL * mVdot_base;
}

void DynamicsMatrix::ComputeY()
{
	MatrixXd TempMat(6 * mnJoint, 6 * mnJoint);
	MatrixXd TempMat_t(6 * mnJoint, 6 * mnJoint);
	VectorXd TempVec = VectorXd::Zero(10 * mnJoint);
	
	VectorXd qddot = VectorXd::Zero(mnJoint);
	for (int i = 0; i < mnJoint; i++)
	{
		qddot[i] = mpRobot->m_joint[i].GetRevoluteJointState().m_rValue[2];
	}

	for (int i = 0; i < mnJoint; i++)
	{
		RowVectorXd e_i_t = RowVectorXd::Zero(mnJoint); e_i_t[i] = 1.0;
		MatrixXd LAe_t = e_i_t*mMatrixA.transpose()*mMatrixL.transpose();
		
		TempMat = mMatrixL * ((mMatrixA * qddot + mMatrix_adA * mMatrixGamma * mMatrixL * mMatrixA * qdot + mVdot_base) * LAe_t + mMatrixA * qdot * LAe_t * mMatrix_adv_t);
		TempMat_t = TempMat.transpose();
		TempMat = (TempMat + TempMat_t) / 2.0;

		for (int j = 0; j < mnJoint; j++)
		{
			// indexes for conversion from G to phi
			int index_i[] = { 0,1,2,0,0,1,1,0,0,3 };
			int index_j[] = { 0,1,2,1,2,2,5,5,4,3 };

			// conversion of each block
			for (int k = 0; k < 10; k++)
			{
				if (k < 3)
					TempVec[10 * j + k] = TempMat(6 * j + index_i[k], 6 * j + index_j[k]);
				else if (k < 6)
					TempVec[10 * j + k] = 2.0 * TempMat(6 * j + index_i[k], 6 * j + index_j[k]);
				else if (k < 9)
					TempVec[10 * j + k] = 2.0 * (TempMat(6 * j + index_i[k], 6 * j + index_j[k]) - TempMat(6 * j + index_j[k] - 3, 6 * j + index_i[k]+3));
				else
					TempVec[10 * j + k] = TempMat(6 * j + index_i[k], 6 * j + index_j[k]) + TempMat(6 * j + index_i[k] + 1, 6 * j + index_j[k] + 1) + TempMat(6 * j + index_i[k] + 2, 6 * j + index_j[k] + 2);

				// revert the sign (skew-symmetricity)
				if (k == 6 || k == 8)
					TempVec[10 * j + k] = -TempVec[10 * j + k];
			}
		}
		mMatrixY.row(i) = TempVec;
	}
}