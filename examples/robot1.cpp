
#include "robot1.h"

robot1::robot1()
{
	buildRobot1();
}
void robot1::buildRobot1()
{
	// design parameter
	SR_real body_diameter = 0.08;
	SR_real body_height = 0.1;
	vector<Inertia> I_list;
	Inertia I0(0.294863503, 0.113500168, 0.25065343, 0.007950225, 0.0000931, 0.000187103, -0.047746296, 1.312477649, -0.007159328, 10.76768767);
	Inertia I1(0.294863503, 0.113500168, 0.25065343, 0.007950225, 0.0000931, 0.000187103, -0.047746296, 1.312477649, -0.007159328, 10.76768767);
	Inertia I2(0.0260684, 0.014722017, 0.019348137, 0.0000135, 0.000117001, -0.0000366, -0.009182943, 0.120340603, 0.059755955, 3.87493756);
	Inertia I3(0.13671601, 0.005883535, 0.139513702, 0.016804342, -0.0000051, 0.00000529, -0.068952728, 0.37398727, 0.0000596, 1.80228141);
	Inertia I4(0.057192689, 0.05716471, 0.003004404, -0.0000147, -0.0000819, 0.0000942, 0.011965126, -0.000550647, 0.31854219, 2.40016804);
	Inertia I5(0.0000559, 0.0000782, 0.0000659, -0.000000256, 0.00000000188, 0.000000833, 0.000011, 0.000632683, 0.000539377, 0.12376019);
	Inertia I6(0.000931067, 0.000498334, 0.000574835, -0.00000148, 0.00000201, 0.000221618, -0.0000513, -0.007118902, 0.010316994, 0.41797364);
	Inertia I7(0.0000385, 0.0000388, 0.0000741, 0.000000191, -0.0000000177, 0.0000000362, -0.00000547, 0.0000112, -0.00022211, 0.06864753);

	I_list.push_back(I0);
	I_list.push_back(I1);
	I_list.push_back(I2);
	I_list.push_back(I3);
	I_list.push_back(I4);
	I_list.push_back(I5);
	I_list.push_back(I6);
	I_list.push_back(I7);


	//Link Geometry
	for (int i = Link1; i < num_links; i++)
	{
		pair<SE3, Vector3d> aa = Mapping_Inertia::principal_decomp_tensor(I_list[i]);

		m_link[i].GetGeomInfo().SetLocalFrame(aa.first);
		m_link[i].GetGeomInfo().SetShape(srGeometryInfo::ELLIPSOID);
		m_link[i].GetGeomInfo().SetDimension(aa.second[0], aa.second[1], aa.second[2]);
		//m_link[i].GetGeomInfo().SetDimension(body_diameter, body_height);
		m_link[i].GetGeomInfo().SetColor(1.0, 0.0, 0.0, 0.2);
		m_link[i].SetInertia(I_list[i]);
		m_link[i].SetDamping(0.01);
	}

	//Joint Assembly
	//joint1
	m_joint[Joint1].SetActType(srJoint::TORQUE);
	m_joint[Joint1].MakePositionLimit(false);
	m_joint[Joint1].SetParentLink(&m_link[BaseLink]);
	m_joint[Joint1].SetChildLink(&m_link[Link1]);
	m_joint[Joint1].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_joint[Joint1].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI / 2), Vec3(0.0, 0.0, 0.0)));
	//joint2
	m_joint[Joint2].SetActType(srJoint::TORQUE);
	m_joint[Joint2].MakePositionLimit(false);
	m_joint[Joint2].SetParentLink(&m_link[Link1]);
	m_joint[Joint2].SetChildLink(&m_link[Link2]);
	m_joint[Joint2].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_joint[Joint2].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI / 2), Vec3(0.0, 0.0, 0.0)));

	m_joint[Joint3].SetActType(srJoint::TORQUE);
	m_joint[Joint3].MakePositionLimit(false);
	m_joint[Joint3].SetParentLink(&m_link[Link2]);
	m_joint[Joint3].SetChildLink(&m_link[Link3]);
	m_joint[Joint3].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_joint[Joint3].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI / 2), Vec3(-0.045, 0.55, 0.0)));

	m_joint[Joint4].SetActType(srJoint::TORQUE);
	m_joint[Joint4].MakePositionLimit(false);
	m_joint[Joint4].SetParentLink(&m_link[Link3]);
	m_joint[Joint4].SetChildLink(&m_link[Link4]);
	m_joint[Joint4].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_joint[Joint4].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI / 2), Vec3(0.045, 0.0, 0.0)));

	m_joint[Joint5].SetActType(srJoint::TORQUE);
	m_joint[Joint5].MakePositionLimit(false);
	m_joint[Joint5].SetParentLink(&m_link[Link4]);
	m_joint[Joint5].SetChildLink(&m_link[Link5]);
	m_joint[Joint5].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_joint[Joint5].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI / 2), Vec3(0.0, 0.3, 0.0)));

	m_joint[Joint6].SetActType(srJoint::TORQUE);
	m_joint[Joint6].MakePositionLimit(false);
	m_joint[Joint6].SetParentLink(&m_link[Link5]);
	m_joint[Joint6].SetChildLink(&m_link[Link6]);
	m_joint[Joint6].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_joint[Joint6].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI / 2), Vec3(0.0, 0.0, 0.0)));

	m_joint[Joint7].SetActType(srJoint::TORQUE);
	m_joint[Joint7].MakePositionLimit(false);
	m_joint[Joint7].SetParentLink(&m_link[Link6]);
	m_joint[Joint7].SetChildLink(&m_link[Link7]);
	m_joint[Joint7].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	m_joint[Joint7].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0), Vec3(0.0, 0.0, -0.06)));


	//link[3].AddCollision(&collision[3]);
	//collision[3].GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
	//collision[3].GetGeomInfo().SetDimension(body_diameter, body_height);
	//collision[3].GetGeomInfo().SetLocalFrame(SE3());


	//// wheel link
	//	for (int i = 0; i < 3; i++)
	//	{

	//		link[i].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
	//		link[i].GetGeomInfo().SetDimension(wheel_diameter, wheel_height);
	//		link[i].GetGeomInfo().SetColor(0.9, 0.2, 0.1);
	//		link[i].UpdateInertia(2);

	//		link[i].AddCollision(&collision[i]);
	//		collision[i].GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
	//		collision[i].GetGeomInfo().SetDimension(wheel_diameter, wheel_height);
	//		collision[i].GetGeomInfo().SetLocalFrame(SE3());
	//				
	//	}

	//	//RIGHT wheel
	//	wheelJoint[0].SetActType(srJoint::TORQUE);
	//	wheelJoint[0].MakePositionLimit(false);
	//	wheelJoint[0].SetParentLink(&link[3]);
	//	wheelJoint[0].SetChildLink(&link[0]);
	//	wheelJoint[0].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI/2,0), Vec3(-0.5*body_diameter, -0.5 * body_diameter, 0.3*body_height)));
	//	wheelJoint[0].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0 , 0.0)));

	//	//LEFT wheel
	//	wheelJoint[1].SetActType(srJoint::TORQUE);
	//	wheelJoint[1].MakePositionLimit(false);
	//	wheelJoint[1].SetParentLink(&link[3]);
	//	wheelJoint[1].SetChildLink(&link[1]);
	//	wheelJoint[1].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI/2, 0.0), Vec3(0.5*body_diameter, -0.5*body_diameter, 0.3*body_height)));
	//	wheelJoint[1].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

	//	//REAR wheel
	//	wheelJoint[2].SetActType(srJoint::PASSIVE);
	//	wheelJoint[2].MakePositionLimit(false);
	//	wheelJoint[2].SetParentLink(&link[3]);
	//	wheelJoint[2].SetChildLink(&link[2]);
	//	wheelJoint[2].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI / 2, 0.0), Vec3(0.0, -0.5*body_diameter, -0.6*body_height)));
	//	wheelJoint[2].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));


	//Base link setting
	m_link[0].SetFrame(EulerZYX(Vec3(0, 0, 0), Vec3(0.0, 0.0, 1.0)));
	SetBaseLink(&m_link[0]);
	SetBaseLinkType(srSystem::FIXED);
	//this->SetSelfCollision(True);

}