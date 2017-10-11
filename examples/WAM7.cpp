#include "WAM7.h"
using namespace vectorOp;

WAM7::WAM7()
	:m_links(num_links), m_joints(num_joints)
{
	AssembleModel();
}

void WAM7::AssembleModel()
{
//	//	kinematic parameter
//	//double body_length[3] = { 0.11, 0.10, 0.09 };
//	//double body_width = 0.025;
//	//double body_thickness = 0.005;
//	//double frame_thickness = 0.005;
//	//double density = 0.368 / 1000 / (body_width * body_thickness * 0.05);
//
//	////////////////////////////////////////////////////////////////////////
//	//								GEOMETRY		
//	////////////////////////////////////////////////////////////////////////
//
	//	body links

	m_links[BaseLink].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	m_links[BaseLink].GetGeomInfo().SetDimension(0.5, 0.5, 0.5);
	m_links[BaseLink].GetGeomInfo().SetColor(0.9, 0.9, 0.9);
	m_links[BaseLink].UpdateInertia(1);
//
//	//for (int i = Link1; i < num_links; i++)
//	//{
//	//	m_links[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);
//	//	//m_links[i].GetGeomInfo().SetColor(0.9f, 0.9f, 0.9f, 0.0f);
//	//	m_links[i].GetGeomInfo().SetDimension(0.5, 0.5, 0.5);
//	//	m_links[i].UpdateInertia(1); // !!!!!!!!!MODIFY!!!!!!!!!
//	//}
//
//	////	forward wing links geometry
//	//for (int l = link_lfw0; l <= link_rfw4; l++)
//	//{
//	//	m_links[l].GetGeomInfo().SetShape(srGeometryInfo::BOX);
//	//	m_links[l].GetGeomInfo().SetDimension(f_wing_length, f_wing_width, wing_thickness);
//	//	m_links[l].GetGeomInfo().SetColor(1.0f, 0.79f, 0.05f);
//	//	m_links[l].UpdateInertia(density * 1);
//	//	//cout << m_links[l].m_Inertia[0] << '\t'<< m_links[l].m_Inertia[1] << '\t' << m_links[l].m_Inertia[2] << '\t' << m_links[l].m_Inertia[9] << endl;
//	//}
//
//	////	joint geometry
//	//for (int j = joint_body1; j <= joint_rbw3; j++)
//	//{
//	//	//m_joints[j] = srRevoluteStiffnessProfileJoint(spline);
//	//	m_joints[j] = srRevoluteStiffnessProfileJoint();
//	//	m_joints[j].m_stiffnessProfile = new tk::spline(spline);
//	//	m_joints[j].SetSpringCoeff(0.1 * magnifying_factor);
//	//	m_joints[j].SetDampingCoeff(0.00002*magnifying_factor);
//	//	m_joints[j].GetGeomInfo().SetDimension(0.01, 0.01, 0.01);
//	//	m_joints[j].SetActType(srJoint::HYBRID);
//
//	//}
//
//	//m_joints[joint_lfw0].SetActType(srJoint::HYBRID);
//	//m_joints[joint_rfw0].SetActType(srJoint::HYBRID);
//	//m_joints[joint_lbw0].SetActType(srJoint::HYBRID);
//	//m_joints[joint_rbw0].SetActType(srJoint::HYBRID);
//
//	////////////////////////////////////////////////////////////////////////
//	//								ASSEMBLE		
//	////////////////////////////////////////////////////////////////////////
//
//
//	////	body
//	//m_joints[joint_body1].SetParentLink(&m_links[link_body0]);
//	//m_joints[joint_body1].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI / 2), Vec3(-body_length[0] / 2, 0, -body_thickness / 2)));
//	//m_joints[joint_body1].SetChildLink(&m_links[link_body1]);
//	//m_joints[joint_body1].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI / 2), Vec3(body_length[1] / 2, 0, -body_thickness / 2)));
//
//	//m_joints[joint_body2].SetParentLink(&m_links[link_body1]);
//	//m_joints[joint_body2].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI / 2), Vec3(-body_length[1] / 2, 0, body_thickness / 2)));
//	//m_joints[joint_body2].SetChildLink(&m_links[link_body2]);
//	//m_joints[joint_body2].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, -SR_PI / 2), Vec3(body_length[2] / 2, 0, body_thickness / 2)));
//
//	////	forward wing frame
//	//for (int lf = link_frame_lfw0, lw = link_lfw0, j = wjoint_lfw0; lf <= link_frame_rfw4; lf++, lw++, j++)
//	//{
//	//	m_wjoints[j].SetParentLink(&m_links[lw]);
//	//	m_wjoints[j].SetParentLinkFrame(EulerZYX(Vec3(0.0), Vec3(0, 0, wing_thickness / 2)));
//	//	m_wjoints[j].SetChildLink(&m_links[lf]);
//	//	m_wjoints[j].SetChildLinkFrame(EulerZYX(Vec3(0.0), Vec3(-f_wing_length / 2, 0, body_thickness / 2 * 0.99)));
//	//}
//
//
//	//	Add collision
//	//for (int i = link_body0; i <= link_body2; i++)
//	//{
//	//	m_collisions[i].GetGeomInfo().SetShape(m_links[i].GetGeomInfo().GetShape());
//	//	m_collisions[i].GetGeomInfo().SetDimension(m_links[i].GetGeomInfo().GetDimension());
//	//	m_links[i].AddCollision(&m_collisions[i]);
//	//}
//
	//	set base link
	m_links[BaseLink].SetFrame(EulerZYX(Vec3(0, 0, SR_PI / 2), Vec3(0.0, 0.0, 3)));
	SetBaseLink(&m_links[BaseLink]);
	SetBaseLinkType(srSystem::FIXED);
}




