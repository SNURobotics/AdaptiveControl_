#include "AdaptiveControl.h"


void AdaptiveControl::ObserveStateFeedbackAndDesiredState(VectorXd q_de_, VectorXd qdot_de_, VectorXd qddot_de_)
{
	
	for (int i = 0; i < mpRobot1->num_joints; i++)
	{
		q[i] = mpRobot1->m_joint[i].GetRevoluteJointState().m_rValue[0];
		qdot[i] = mpRobot1->m_joint[i].GetRevoluteJointState().m_rValue[1];
		qddot[i] = mpRobot1->m_joint[i].GetRevoluteJointState().m_rValue[2];
	}
	q_de = q_de_;
	qdot_de = qdot_de_;
	qddot_de = qddot_de_;
	switch (mControlType)
	{
	case PassivityBasedControl:
		v = qdot_de - Lambda_PBC*(q - q_de);
		r = qdot - v;
		a = qddot_de - Lambda_PBC*(qdot - qdot_de) - lambda_c_PBC*r;
		break;
	case ComputedTorqueControl:
		break;
	default:
		break;
	}


}
void AdaptiveControl::AdaptParameter()
{
	vector<MatrixXd> G_current(mpRobot1->num_joints);
	//VectorXd phi_current = VectorXd::Zero(10 * mpRobot1->num_joints);
	VectorXd phi_update = VectorXd::Zero(10 * mpRobot1->num_joints);
	//vector<MatrixXd> P_current(mpRobot1->num_joints);
	vector<MatrixXd> P_update(mpRobot1->num_joints);
	vector<MatrixXd> co_P_current(mpRobot1->num_joints);
	VectorXd b_adapt;

	switch (mAdaptationType)
	{
	case Euclidean:
		
		for (int j = 0; j < mpRobot1->num_joints; j++)
		{
			G_current[j] = MatrixXd::Zero(6,6);
			mvpEstimatedInertia[j]->ToArray(G_current[j].data());
			phi_current.block<10, 1>(10 * j, 0) = TensorMat2TensorVec(G_current[j]);
		}
		b_adapt = (mDynamicsMatrix->mMatrixY).transpose() * r;

		phi_update = phi_current - time_step * Sigma_Euc_inv * b_adapt;

		for (int j = 0; j < mpRobot1->num_joints; j++)
		{
			mvpEstimatedInertia[j]->SetMass((SR_real) phi_update[10*j+9]);
			mvpEstimatedInertia[j]->SetOffset(Vec3((SR_real)phi_update[10 * j + 6], (SR_real)phi_update[10 * j + 7], (SR_real)phi_update[10 * j + 8]));
			mvpEstimatedInertia[j]->SetAngularMoment((SR_real)phi_update[10 * j + 0], (SR_real)phi_update[10 * j + 1], (SR_real)phi_update[10 * j + 2], (SR_real)phi_update[10 * j + 3], (SR_real)phi_update[10 * j + 4], (SR_real)phi_update[10 * j + 5]);
		}
		break;
	case Bregman:
		b_adapt = (mDynamicsMatrix->mMatrixY).transpose() * r;
		for (int j = 0; j < mpRobot1->num_joints; j++)
		{
			G_current[j] = MatrixXd::Zero(6, 6);
			mvpEstimatedInertia[j]->ToArray(G_current[j].data());
			phi_current.block<10, 1>(10 * j, 0) = TensorMat2TensorVec(G_current[j]);
			//cout << "222" << endl;
			P_current[j] = TensorMat2MomentMat(G_current[j]);
			co_P_current[j] = coTensorVec2coMomentMat(b_adapt.block<10,1>(10*j,0));

			P_update[j] = P_current[j] - time_step * (1 / gamma_Br) * P_current[j] * co_P_current[j] * P_current[j];
		}
		for (int j = 0; j < mpRobot1->num_joints; j++)
		{
			//cout << "333" << endl;
			mvpEstimatedInertia[j]->SetMass((SR_real)P_update[j](3,3));
			mvpEstimatedInertia[j]->SetOffset(Vec3((SR_real)P_update[j](0,3), (SR_real)P_update[j](1, 3), (SR_real)P_update[j](2, 3)));
			mvpEstimatedInertia[j]->SetAngularMoment((SR_real)P_update[j](1, 1)+ P_update[j](2, 2), (SR_real)P_update[j](0, 0)+ P_update[j](2, 2), (SR_real)P_update[j](0, 0)+ P_update[j](1, 1), (SR_real)-P_update[j](0, 1), (SR_real)-P_update[j](0, 2), (SR_real)-P_update[j](1, 2));
		}


		break;
	default:
		break;
	}


}
void AdaptiveControl::ApplyTorque()
{
	switch (mControlType)
	{
		case PassivityBasedControl:
			K_PBC = lambda_c_PBC * mDynamicsMatrix->M;
			mTorqueInput = mDynamicsMatrix->M * a + mDynamicsMatrix->C * v + mDynamicsMatrix->N - K_PBC*r;
			break;
		case ComputedTorqueControl:
			break;
		default:
			break;
	}
		
	for (int j = 0; j < mpRobot1->num_joints; j++)
	{
		mpRobot1->m_joint[j].m_State.m_rCommand = mTorqueInput[j];
	}
	//cout << "AppliedTorque:" << mTorqueInput << endl;
}
double AdaptiveControl::GetLyapunovF()
{
	double LyapunovF = 0;
	switch (mControlType)
	{
	case PassivityBasedControl:
		LyapunovF = LyapunovF + 0.5 * r.transpose() * mDynamicsMatrix->M_true * r + (q - q_de).transpose() * Lambda_PBC*K_PBC * (q - q_de);
		break;
	case ComputedTorqueControl:
		break;
	default:
		break;
	}
	switch (mAdaptationType)
	{
	case Euclidean:
		//LyapunovF = LyapunovF + 0.5 * (phi_current - phi_true).transpose() * Sigma_Euc * (phi_current - phi_true);
		break;
	case Bregman:
		break;
	default:
		break;
	}

	return LyapunovF;
}
AdaptiveControl::AdaptiveControl(robot1* p_robot1, SR_real time_step_, vector<Inertia*> vpEstimatedInertia, ControlType ControlType, AdaptationType AdaptationType)
	:mpRobot1(p_robot1), mvpEstimatedInertia(vpEstimatedInertia)
{
	time_step = (double)time_step_;
	phi_current = VectorXd::Zero(mpRobot1->num_joints*10);
	P_current.resize(mpRobot1->num_joints);
	phi_true = VectorXd::Zero(mpRobot1->num_joints * 10);
	for (int j = 0; j < mpRobot1->num_joints; j++)
	{
		//cout << "111" << endl;
		P_current[j] = MatrixXd::Zero(4, 4);

		MatrixXd G_temp = MatrixXd::Zero(6, 6);
		mpRobot1->m_link[j + 1].m_Inertia.ToArray(G_temp.data());
		phi_true.block<10, 1>(10 * j, 0) = TensorMat2TensorVec(G_temp);
	}

	q = VectorXd::Zero(mpRobot1->num_joints);
	qdot = VectorXd::Zero(mpRobot1->num_joints);
	qddot = VectorXd::Zero(mpRobot1->num_joints);
	q_de = VectorXd::Zero(mpRobot1->num_joints);
	qdot_de = VectorXd::Zero(mpRobot1->num_joints);
	qddot_de = VectorXd::Zero(mpRobot1->num_joints);
	a = VectorXd::Zero(mpRobot1->num_joints);
	v = VectorXd::Zero(mpRobot1->num_joints);
	r = VectorXd::Zero(mpRobot1->num_joints);
	mTorqueInput = VectorXd::Zero(mpRobot1->num_joints);
	mControlType = ControlType; 
	mAdaptationType = AdaptationType;
	cout << q << endl;
	switch (mAdaptationType)
	{
	case Euclidean:
		Sigma_Euc = MatrixXd::Zero(10 * mpRobot1->num_joints, 10 * mpRobot1->num_joints);
		//Standard euclidean metric
			Sigma_Euc = 1000 * MatrixXd::Identity(10* mpRobot1->num_joints,10* mpRobot1->num_joints);
		//Mahalanobis metric
			//for (int i = 0; i < mpRobot1->num_joints; i++)
			//{
			//	Sigma_Euc.block<10, 10>(10 * i, 10 * i) = 0.5 * TensorVec2RiemMetric(phi_true.block<10, 1>(10 * i, 0));
			//}

		Sigma_Euc_inv = Sigma_Euc.inverse();
		

		break;
	case Bregman:
		gamma_Br = 0.5;
		break;
	default:
		break;
	}
	switch (mControlType)
	{
	case PassivityBasedControl:
		lambda_c_PBC = 50.0;
		Lambda_PBC = lambda_c_PBC*MatrixXd::Identity(mpRobot1->num_joints, mpRobot1->num_joints);
		K_PBC = lambda_c_PBC*MatrixXd::Identity(mpRobot1->num_joints, mpRobot1->num_joints);
		break;
	case ComputedTorqueControl:
		break;
	default:
		break;
	}

}
AdaptiveControl::AdaptiveControl()
{

}
AdaptiveControl::~AdaptiveControl()
{
	//delete[] mpTorqueInput;
}