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

	vector<Inertia*> mvpEstimatedInertia;
	robot1* mpRobot1;
	//States
	VectorXd q;
	VectorXd qdot;
	VectorXd qddot;
	VectorXd q_de;
	VectorXd qdot_de;
	VectorXd qddot_de;
	VectorXd a;
	VectorXd v;
	VectorXd r;

	VectorXd mTorqueInput;
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
	
	// PBC gains
	MatrixXd Lambda_PBC;
	MatrixXd K_PBC;

	//Bregman gains
	double gamma_Br;


	void ObserveStateFeedbackAndDesiredState(VectorXd q_de_, VectorXd qdot_de_, VectorXd qddot_de_);
	void AdaptParameter();	// update intertia matrices.
	void ApplyTorque();		// update torques.
	double GetLyapunovF();

	AdaptiveControl();
	AdaptiveControl(robot1* p_robot1, vector<Inertia*> I0, ControlType ControlType, AdaptationType AdaptationType);
	~AdaptiveControl();
};