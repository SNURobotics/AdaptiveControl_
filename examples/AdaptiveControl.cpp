#include "AdaptiveControl.h"

AdaptiveControl::AdaptiveControl()
{

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


	//GetStatePtr()->SetCommand(real * mp_TorqueInput) 
}
void AdaptiveControl::ApplyTorque()
{
	switch (mControlType)
	{
		case PassivityBasedControl:
			break;
		case ComputedTorqueControl:
			break;
		default:
			break;
	}
		
		
	//GetStatePtr()->SetCommand(real * mp_TorqueInput) 
}

AdaptiveControl::AdaptiveControl(robot1* p_robot1, vector<Inertia*> vpEstimatedInertia, ControlType ControlType, AdaptationType AdaptationType)
	:mpRobot1(p_robot1), mvpEstimatedInertia(vpEstimatedInertia)
{
	SR_real* mp_TorqueInput = new SR_real[mpRobot1->num_joints];
	mControlType = ControlType; 
	mAdaptationType = AdaptationType;
}

AdaptiveControl::~AdaptiveControl()
{
	delete[] mpTorqueInput;
}