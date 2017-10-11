#pragma once

#include "robot1.h"
#include "Mapping_Inertia.h"
#include "AdaptiveControl.h"
#include <Eigen\Core>
#include "LieGroup/LieGroup.h"

using namespace Eigen;
using namespace std;

class AdaptiveControl;

class DynamicsMatrix
{
public: // variables

	robot1* mpRobot;
	AdaptiveControl* mpAdaptiveControl;

	int mnJoint;
	vector<Inertia*> mvpEstimatedInertia;
	vector<SE3> mvCurrentSE3;
	vector<se3> mvCurrentV;
	vector<se3> mvA;

	VectorXd vdot0;//
	VectorXd qdot;//
	MatrixXd mMatrixA;//
	MatrixXd mMatrixL;//
	MatrixXd mMatrixG;//
	MatrixXd mMatrixGamma;//
	MatrixXd mMatrixCbar;//
	MatrixXd mVdot_base;//
	MatrixXd mMatrix_adA;//
	MatrixXd mMatrix_adv_t;//
	//MatrixXd MatrixY;

	MatrixXd M;
	MatrixXd C;
	MatrixXd N;

public: // functions

	DynamicsMatrix(robot1* pRobot, AdaptiveControl* pAdaptiveControl);

	void UpdateMatrices();
	MatrixXd BigAd(SE3 T);
	MatrixXd SmallAd(se3 V);
	MatrixXd CrossMatrix(Vec3 w);
	// MatrixXd ComputeA(); // A is fixed.
	void ComputeL();
	void ComputeG();
	void ComputeGamma();
	void ComputeCbar();
	void Compute_adA();
	void Compute_adv_t();
	void ComputeVdot_base();

	void MassMatrix();
	void CoriolisMatrix();
	void GravitationalVector();

private:
	
};