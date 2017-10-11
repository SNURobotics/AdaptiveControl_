#include "utils.h"

#define INV_KIN_TOL 1E-6

RMatrix srUtils::getBodyJacobian(srLink * _link, SE3 offset /* = SE3()*/)
{
	// assume joint value is already entered as input
	srLink* curLink = new srLink();
	curLink = _link;
	int dof = 0;
	vector<se3> curScrew(0);
	while (!curLink->m_IsBaseLink)
	{
		if (curLink->m_ParentJoint->GetType() == srJoint::JOINTTYPE::REVOLUTE)
			curScrew.push_back(Ad((_link->GetFrame() % curLink->m_ParentJoint->GetFrame()), se3(0, 0, 1, 0, 0, 0)));
		else if (curLink->m_ParentJoint->GetType() == srJoint::JOINTTYPE::PRISMATIC)
			curScrew.push_back(Ad((_link->GetFrame() % curLink->m_ParentJoint->GetFrame()), se3(0, 0, 0, 0, 0, 1)));
		curLink = curLink->m_ParentLink;
	}
	if (curScrew.size() == 0)
		return RMatrix();
	RMatrix Jac(6, curScrew.size());
	for (unsigned int i = 0; i < curScrew.size(); i++)
	{
		for (int j = 0; j < 6; j++)
			Jac[6 * i + j] = curScrew[curScrew.size() - 1 - i][j];
	}
	return Jac;
}

RMatrix srUtils::inverseKinematics(srLink * _link, SE3 _goal, RMatrix& q0, SE3 offset /* = SE3()*/)
{
	// get system info
	srLink* curLink = new srLink();
	curLink = _link;
	vector<srJoint*> tempJoints(0);
	while (!curLink->m_IsBaseLink)
	{
		if (curLink->m_ParentJoint->GetType() == srJoint::JOINTTYPE::REVOLUTE || curLink->m_ParentJoint->GetType() == srJoint::JOINTTYPE::PRISMATIC)
			tempJoints.push_back(curLink->m_ParentJoint);
		curLink = curLink->m_ParentLink;
	}
	assert(q0.RowSize() == tempJoints.size() && "check the size of initial condition");
	vector<srJoint*> robotJoints(tempJoints.size());
	for (unsigned int i = 0; i < tempJoints.size(); i++)
		robotJoints[i] = tempJoints[tempJoints.size() - 1 - i];
	// solve inverse kinematics
	double delta = 1.;
	double step_size = 0.01;
	se3 error;
	RMatrix error_vec(6);
	RMatrix q = q0;
	while (delta > INV_KIN_TOL)
	{
		// update joint value
		for (int i = 0; i < q0.RowSize(); i++)
		{
			if (robotJoints[i]->GetType() == srJoint::JOINTTYPE::REVOLUTE)
				((srRevoluteJoint*)robotJoints[i])->m_State.m_rValue[0] = q[i];
			else
				((srPrismaticJoint*)robotJoints[i])->m_State.m_rValue[0] = q[i];
		}
		SE3 initPos = _link->GetFrame();
		_link->m_pSystem->KIN_UpdateFrame_All_The_Entity();
		SE3 aftePos = _link->GetFrame();
		// find error
		error = Log((_link->GetFrame()*offset) % _goal);
		delta = 0.0;
		for (int i = 0; i < 6; i++)
		{
			error_vec[i] = error[i];
			delta += error[i] * error[i];
		}
		RMatrix Jb = getBodyJacobian(_link, offset);
		
		//RMatrix printVal = Jb;
		//for (int i = 0; i < printVal.ColSize()*printVal.RowSize(); i++)
		//	printf("%f \n", printVal[i]);
		RMatrix JbpInv = pInv(Jb);
		q += step_size*JbpInv*error_vec;
	}

	return q;
}


