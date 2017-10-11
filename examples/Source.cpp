#include "LieGroup\rmatrix3.h"
#include "common\utils.h"

srSystem* gSystem = new srSystem();
srLink gLink[20];
srRevoluteJoint gJoint[20];
srSpace* gSpace = new srSpace();

void assembly(int dof)
{
	gSystem->SetBaseLink(&gLink[0]);
	for (int i = 0; i < dof; i++)
	{
		gJoint[i].SetParentLink(&gLink[i]);
		gJoint[i].SetParentLinkFrame(SE3(Vec3(0., 1., 0.)));
		gJoint[i].SetChildLink(&gLink[i + 1]);
		gJoint[i].SetChildLinkFrame(SE3(Vec3(0., -1., 0.)));
	}
	gLink[0].SetFrame(SE3());
	gSystem->SetBaseLinkType(srSystem::BASELINKTYPE::FIXED);
	gSpace->AddSystem(gSystem);
	gSpace->DYN_MODE_PRESTEP();
}
int main(int argc, char **argv)
{
	RMatrix A(4, 2);
	A[0] = 1.;
	A[1] = 2.;
	A[2] = 3.;
	A[3] = 4.;
	A[4] = 5.;
	A[5] = 6.;
	A[6] = 7.;
	A[7] = 8.;

	RMatrix U, S, V;
	SVD(A, U, S, V);
	printf("U: (%d, %d)\n", U.RowSize(), U.ColSize());
	for (int i = 0; i < U.ColSize()*U.RowSize(); i++)
		printf("%f \n", U[i]);
	
	printf("S: (%d, %d))\n", S.RowSize(), S.ColSize());
	for (int i = 0; i < S.RowSize(); i++)
		printf("%f \n", S[i]);
	
	printf("V: (%d, %d)\n", V.RowSize(), V.ColSize());
	for (int i = 0; i < V.ColSize()*V.RowSize(); i++)
		printf("%f \n", V[i]);
	
	printf("-------------\n");
	printf("%d, %d\n", S.ColSize(), S.RowSize());


	RMatrix Apinv = pInv(A);
	printf("Apinv(%d, %d): \n", Apinv.RowSize(), Apinv.ColSize());
	for (int i = 0; i < Apinv.ColSize()*Apinv.RowSize(); i++)
		printf("%f \n", Apinv[i]);
	

	//RMatrix Acheck = 
	

	int dof = 3;
	assembly(dof);


	SE3 Goal = SE3(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(3., 3., 0.)));
	cout << Goal << endl;
	RMatrix J = srUtils::getBodyJacobian(&gLink[dof]);
	printf("J: \n");
	for (int i = 0; i < J.ColSize()*J.RowSize(); i++)
	{
		printf("%f \n", J[i]);
	}


	RMatrix q(dof);
	q[0] = 0.123123;
	q[1] = 0.6204;
	q[2] = -0.12341234;
	for (int i = 0; i < dof; i++)
		gJoint[i].m_State.m_rValue[0] = q[i];
	gSystem->KIN_UpdateFrame_All_The_Entity();
	cout << gLink[dof].GetFrame() << endl;
	Goal = gLink[dof].GetFrame();
	RMatrix q0(dof);
	q0[1] = -0.3;
	printf("q0: \n");
	for (int i = 0; i < q0.ColSize()*q0.RowSize(); i++)
	{
		printf("%f \n", q0[i]);
	}
	q = srUtils::inverseKinematics(&gLink[dof], Goal, q0);
	printf("q: \n");
	for (int i = 0; i < q.ColSize()*q.RowSize(); i++)
	{
		printf("%f \n", q[i]);
	}
	for (int i = 0; i < dof; i++)
		gJoint[i].m_State.m_rValue[0] = q[i];
	gSystem->KIN_UpdateFrame_All_The_Entity();
	cout << gLink[dof].GetFrame() << endl;



	return 0;
}