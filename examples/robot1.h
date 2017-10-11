#ifndef ROBOT1_H
#define ROBOT1_H

#include "srDyn/srSpace.h"
#include "srDyn/srSystem.h"
#include <iostream>
#include "Mapping_Inertia.h"
#include <Eigen\Core>

using namespace Eigen;
using namespace std;

class robot1 : public srSystem
{
public:

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

	srLink m_link[num_links];
	srRevoluteJoint m_joint[num_joints];
	//srCollision collision[4];


public:
	// constructor
	robot1();
	void buildRobot1();
};

#endif //ROBOT1_H