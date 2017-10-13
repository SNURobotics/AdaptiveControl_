#include "AdaptiveControl.h"


void AdaptiveControl::GetStateFeedbackAndDesiredState(VectorXd q_de_, VectorXd qdot_de_, VectorXd qddot_de_)
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
		a = qddot_de - Lambda_PBC*(qdot - qdot_de);
		r = qdot - v;
		break;
	case ComputedTorqueControl:
		break;
	default:
		break;
	}


}
void AdaptiveControl::AdaptParameter()
{
	switch (mAdaptationType)
	{
	case Euclidean:
		break;
	case Bregman:
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
			
			mTorqueInput = mDynamicsMatrix->M * a + mDynamicsMatrix->C * v + mDynamicsMatrix->N - K_PBC*r;
			break;
		case ComputedTorqueControl:
			break;
		default:
			break;
	}
		
	for (int j = 0; j < 7; j++)
	{
		mpRobot1->m_joint[j].m_State.m_rCommand = mTorqueInput[j];
	}
	cout << "AppliedTorque:" << mTorqueInput << endl;
}

AdaptiveControl::AdaptiveControl(robot1* p_robot1, vector<Inertia*> vpEstimatedInertia, ControlType ControlType, AdaptationType AdaptationType)
	:mpRobot1(p_robot1), mvpEstimatedInertia(vpEstimatedInertia)
{
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
		break;
	case Bregman:
		gamma_Br = 1.0;
		break;
	default:
		break;
	}
	switch (mControlType)
	{
	case PassivityBasedControl:
		Lambda_PBC = 100*MatrixXd::Identity(mpRobot1->num_joints, mpRobot1->num_joints);
		K_PBC = 20*MatrixXd::Identity(mpRobot1->num_joints, mpRobot1->num_joints);
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