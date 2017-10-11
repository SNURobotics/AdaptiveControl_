#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "LieGroup/LieGroup.h"
#include <vector>
#include <math.h>
#include "Mapping_Inertia.h"
#include "robot1.h"
#include "DynamicsMatrix.h"

class DynamicsMatrix;

class AdaptiveControl
{
public:
	AdaptiveControl();

	vector<Inertia*> mvpEstimatedInertia;
	robot1* mpRobot1;
	SR_real* mpTorqueInput;
	DynamicsMatrix* mDynamicsMatrix;
	// SR_real mNoise;

	const enum ControlType
	{
		ComputedTorqueControl,
		PassivityBasedControl
	};
	const enum AdaptationType
	{
		Euclidean,
		Bregman
	};

	ControlType mControlType; 
	AdaptationType mAdaptationType;
	
	void AdaptParameter();	// update intertia matrices.
	void ApplyTorque();		// update torques.

	AdaptiveControl(robot1* p_robot1, vector<Inertia*> I0, ControlType ControlType, AdaptationType AdaptationType);
	~AdaptiveControl();
};