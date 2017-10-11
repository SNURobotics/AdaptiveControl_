#pragma once

#include <vector>

#include <srDyn\srSpace.h>
#include <srDyn\srSystem.h>
#include <srDyn\srRevoluteJoint.h>
#include <common\utils.h>
#include <Eigen\Core>

#include "vectorOp.hpp"

using namespace std;
using namespace Eigen;

class WAM7 : public srSystem
{
	WAM7();
	~WAM7() {};


	const enum LinkID
	{
		BaseLink,
		Link1,
		Link2,
		Link3,
		Link4,
		Link5,
		Link6,
		Link7,
		num_links
	};
	const enum R_Joint_ID
	{
		Joint1,
		Joint2,
		Joint3,
		Joint4,
		Joint5,
		Joint6,
		Joint7,
		num_joints
	};

	const enum W_Joint_ID
	{
		num_wjoints
	};

	//

	////protected:
	void	AssembleModel();


	////srSystem					m_system;
	vector<srRevoluteJoint>		m_joints;
	////vector<srWeldJoint>			m_wjoints;
	vector<srLink>				m_links;
	////vector<srCollision>			m_collisions;
};