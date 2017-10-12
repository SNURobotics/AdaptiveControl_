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

	mvCurrentSE3.resize(mnJoint+1); // # of link = # of joint + 1. (base link)
	mvCurrentV.resize(mnJoint + 1);
	mvA.resize(mnJoint + 1);

	// A matrix
	for (int i = 0; i < mnJoint; i++)
	{
		Vec3 JointAxis = mpRobot->m_link[i].m_Frame.GetZ(); // i-th joint axis = z-axis of (i-1)-th link frame
		MatrixXd w(3,1); w << JointAxis[0], JointAxis[1], JointAxis[2];
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
	for (int i = 0; i < mnJoint+1; i++) // # of link = # of joint + 1. (base link)
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
	Compute_adv_t();
	ComputeVdot_base();

	MassMatrix();
	CoriolisMatrix();
	GravitationalVector();

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

MatrixXd DynamicsMatrix::SmallAd(se3 V)
{
	MatrixXd matSmallAd(6, 6);
	matSmallAd = MatrixXd::Constant(6, 6, 0.0);

	double array[6];
	V.ToArray(array);
	Vec3 w(array[0], array[1], array[2]), v(array[3], array[4], array[5]);

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
	tmp = mMatrix_adv_t * mMatrixG;
	mMatrixCbar = tmp - tmp.transpose();
}


void DynamicsMatrix::Compute_adA()
{
	for (int i = 0; i < mnJoint; i++)
	{
		se3 A_i = qdot[i] * mvA[i];
		mMatrix_adA.block<6, 6>(6 * i, 6 * i) = -SmallAd(A_i);
	}
}

void DynamicsMatrix::Compute_adv_t()
{
	for (int i = 0; i < mnJoint; i++)
	{
		MatrixXd ad_v;
		ad_v = SmallAd(mvCurrentV[i+1]);
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

	