#include <iostream>
#include <vector>
#include <omp.h>

// Include necessary header files.
#include "Renderer/SimpleViewer.h"    // for rendering
#include "srDyn/srSpace.h"            // for dynamics    
#include "srDyn/srSystem.h"
#include "robot1.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "Mapping_Inertia.h"
#include "AdaptiveControl.h"
#include "DynamicsMatrix.h"

// If you want to use some predefined rendering functions,
// include below.
#include "srg/srgGeometryDraw.h"		// for User rendering

// Ground
#include "Ground.h"

using namespace std;
using namespace Eigen;
using namespace Mapping_Inertia;

class robot1;
// Get srSimpleViewer instance. (singleton)
srSimpleViewer& gViewer = srSimpleViewer::GetInstance();

// Declare srSpace instance.
srSpace gSpace;

robot1 Robot;
AdaptiveControl* AdaptiveController; 
DynamicsMatrix* DynMatrices;
//srSystem wam7_robot;

/////////////ERASE HERE///////////////

vector<Inertia*> I_list;
Inertia I1(0.294863503, 0.113500168, 0.25065343, 0.007950225, 0.0000931, 0.000187103, -0.047746296, 1.312477649, -0.007159328, 10.76768767);
Inertia I2(0.0260684, 0.014722017, 0.019348137, 0.0000135, 0.000117001, -0.0000366, -0.009182943, 0.120340603, 0.059755955, 3.87493756);
Inertia I3(0.13671601, 0.005883535, 0.139513702, 0.016804342, -0.0000051, 0.00000529, -0.068952728, 0.37398727, 0.0000596, 1.80228141);
Inertia I4(0.057192689, 0.05716471, 0.003004404, -0.0000147, -0.0000819, 0.0000942, 0.011965126, -0.000550647, 0.31854219, 2.40016804);
Inertia I5(0.0000559, 0.0000782, 0.0000659, -0.000000256, 0.00000000188, 0.000000833, 0.000011, 0.000632683, 0.000539377, 0.12376019);
Inertia I6(0.000931067, 0.000498334, 0.000574835, -0.00000148, 0.00000201, 0.000221618, -0.0000513, -0.007118902, 0.010316994, 0.41797364);
Inertia I7(0.0000385, 0.0000388, 0.0000741, 0.000000191, -0.0000000177, 0.0000000362, -0.00000547, 0.0000112, -0.00022211, 0.06864753);
////////////////////////////////////

// >>>>> DECLARE VARIABLES HERE. <<<<<
// Ground
Ground gGround;

// For modeling your robots(systems), we need many codes. 
// It would be too long to be in main() function. 
// So we this many codes put into the function named User_Modeling().
void User_Modeling();

// For simulation, there are some necessary settings.
// Following function will do that.
void User_SimulationSetting();

// These functions are for simulation start and stop.
// DO NOT MODIFY THEM.
void User_Simulation_Go_One_Step();    // simulation main loop
void User_Simulation_Pause();            // empty function
void User_CBFunc_Run_DYN(void); 
void User_CBFunc_Pause_DYN(void);

// If you want to control your mobile robot, we need a contorl function.
// And this function must be registered at srSpace.
void User_CBFunc_ControlLoop();    // User control loop.

// Callback function for user rendering
//   put your code to render what you want. (OpenGL functions avaliable.)
void User_CBFunc_Render(void* pvData);

// Callback function for user key binding
void User_CBFunc_KeyFunc(char key, void* pvData);

// >>>>> DECLARE CALLBACK FUNCTIONS HERE. <<<<<

////////////////////////////////////////////////////////////////
// main
////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	
    // STEP 1: Viewer initialization
    gViewer.Init(&argc, argv, "Template");
	//default: gViewer.Init(&argc, argv);

    // STEP 2: Robot Modeling
    User_Modeling();
    
    // STEP 3: Simulation setting
    User_SimulationSetting();
	

	///////////////////ERASE HERE////////////////////////
	Matrix4d BaseFrame;
	Robot.m_BaseLink->m_Frame.ToArray(BaseFrame.data());
	cout << "BaseFrame: " << BaseFrame << endl;
	Inertia I0(1,1,1,1,1,1,0.5,0.5,0.5,2);
	pair<SE3, Vector3d> aa = principal_decomp_tensor(I0);

	

	///////////////////////////////////////////////////////



    // STEP 4: Run window view.
    gViewer.Run();

	delete DynMatrices;

    return 0;
}

void User_Modeling()
{
	gSpace.AddSystem(gGround.BuildGround());
	
	// >>>>> WRITE YOUR MODELING CODE HERE. >>>>>
	gSpace.AddSystem(&Robot);

	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	////- Space
	// Set simulation time step.
	gSpace.SetTimestep(0.0005);
	//gSpace.SetTimestep(0.0000005);
	// Set gravity
	gSpace.SetGravity(0.0, 0.0, -9.8);

	// Set number of sub-step for rendering
	gSpace.SetNumberofSubstepForRendering(50);
}

// Simulation Setting.
void User_SimulationSetting()
{


    // Initialize for dynamics simulation.
    gSpace.DYN_MODE_PRESTEP();

	I_list.push_back(&I1);
	I_list.push_back(&I2);
	I_list.push_back(&I3);
	I_list.push_back(&I4);
	I_list.push_back(&I5);
	I_list.push_back(&I6);
	I_list.push_back(&I7);

	AdaptiveController = new AdaptiveControl(&Robot, I_list, AdaptiveControl::ControlType::PassivityBasedControl, AdaptiveControl::AdaptationType::Bregman);
	DynMatrices = new DynamicsMatrix(&Robot, AdaptiveController);
	AdaptiveController->mDynamicsMatrix = DynMatrices;

	///////////////////// erase here //////////////////////////

	//DynMatrices->mvpEstimatedInertia = I_list;
	
	////////////////////////////////////////////

	// Set user control loop function.
	gSpace.SET_USER_CONTROL_FUNCTION(User_CBFunc_ControlLoop);

    // Set target space to render.
    // Let srSimpleRenderer know what you want to draw on screen.
    gViewer.SetTarget(&gSpace);

	// Additional step: Set your user-render function.
	gViewer.SetUserRenderFunc(User_CBFunc_Render, NULL);

    // Key mapping
    // One key can have only one function, but one function 
    // can be connected with more than one key.
    gViewer.SetKeyFunc(User_CBFunc_Run_DYN, 'P');
    gViewer.SetKeyFunc(User_CBFunc_Run_DYN, 'p');

    gViewer.SetKeyFunc(User_CBFunc_Pause_DYN, 'O');
    gViewer.SetKeyFunc(User_CBFunc_Pause_DYN, 'o');

	gViewer.SetUserKeyFunc(User_CBFunc_KeyFunc, NULL);
}

// >>>>> WRITE YOUR CONTROL CODE HERE. <<<<<
void User_CBFunc_ControlLoop()
{
	//Set Reference Trajectory
	double w = 1;
	double A = SR_PI / 3;
	double k1 = 100;
	double k0 = k1*k1/4;
	VectorXd q_de = VectorXd::Zero(7);
	VectorXd qdot_de = VectorXd::Zero(7);
	VectorXd qddot_de = VectorXd::Zero(7);
	for (int j = 0; j < 7; j++)	
	{
		q_de[j] = A*sin(w*gSpace.m_Simulation_Time);
		qdot_de[j] = A*w*cos(w*gSpace.m_Simulation_Time);
		qddot_de[j] = -A*w*w*sin(w*gSpace.m_Simulation_Time);
		//q_de[j] = A*w*gSpace.m_Simulation_Time;
		//qdot_de[j] = A*w;
		//qddot_de[j] = 0.0;
	}
	DynMatrices->UpdateMatrices();

	//AdaptiveController->GetStateFeedbackAndDesiredState(q_de, qdot_de, qddot_de);
	////AdaptiveControl.AdaptParameter();
	//AdaptiveController->ApplyTorque(); // using updated inertia.
	VectorXd q = VectorXd::Zero(7);
	VectorXd qdot = VectorXd::Zero(7);
	VectorXd qddot = VectorXd::Zero(7);
	VectorXd a = VectorXd::Zero(7);
	VectorXd v = VectorXd::Zero(7);
	VectorXd r = VectorXd::Zero(7);
	MatrixXd Lambda_PBC = 1 * MatrixXd::Identity(7, 7);
	MatrixXd K_PBC = 10 * DynMatrices->M;
	for (int i = 0; i < 7; i++)
	{
		q[i] = Robot.m_joint[i].GetRevoluteJointState().m_rValue[0];
		qdot[i] = Robot.m_joint[i].GetRevoluteJointState().m_rValue[1];
		qddot[i] = Robot.m_joint[i].GetRevoluteJointState().m_rValue[2];
	}
	v = qdot_de - Lambda_PBC*(q - q_de);
	a = qddot_de - Lambda_PBC*(qdot - qdot_de);
	r = qdot - v;
	VectorXd u = VectorXd::Zero(7);
	u = DynMatrices->M * a + DynMatrices->C * v + DynMatrices->N - K_PBC*r;
	for (int j = 0; j < 7 ;j++)
	{
		Robot.m_joint[j].m_State.m_rCommand = u[j];
	}

	VectorXd phi(DynMatrices->mnJoint * 10, 1);
	for (int i = 0; i < DynMatrices->mnJoint; i++)
	{
		MatrixXd G_i(6, 6);
		VectorXd phi_i(10, 1);
		I_list[i]->ToArray(G_i.data());
		phi_i = TensorMat2TensorVec(G_i);
		phi.block<10, 1>(10 * i, 0) = phi_i;

		MatrixXd tempvel(6, 1);  DynMatrices->mvCurrentV[i].ToArray(tempvel.data());
		//cout << "vel error: " << endl << tempvel - (DynMatrices->mMatrixL * DynMatrices->mMatrixA * qdot).block<6,1>(6*i,0) << endl;
	}
	cout << "DynMatrices->mMatrixY * phi - u= " << endl << DynMatrices->mMatrixY * phi - (DynMatrices->M * qddot + DynMatrices->C * qdot + DynMatrices->N) << endl; // DynMatrices->mMatrixY * phi - u


	//MatrixXd K0 = k0*MatrixXd::Identity(7, 7);
	//MatrixXd K1 = k1*MatrixXd::Identity(7, 7);
	//VectorXd u = VectorXd::Zero(7);
	//

	//// torque from dynamicMatrix
	//DynMatrices->UpdateMatrices();

	////int i = 3;
	//VectorXd q = VectorXd::Zero(7);
	//VectorXd qdot = VectorXd::Zero(7);
	//VectorXd qddot = VectorXd::Zero(7);
	//for (int j = 0; j < 7; j++)	
	//{
	//	q_de[j] = a*sin(w*gSpace.m_Simulation_Time);
	//	qdot_de[j] = a*w*cos(w*gSpace.m_Simulation_Time);
	//	qddot_de[j] = -a*w*w*sin(w*gSpace.m_Simulation_Time);
	//	//q_de[j] = a*w*gSpace.m_Simulation_Time;
	//	//qdot_de[j] = a*w;
	//	//qddot_de[j] = 0;

	//	q[j] = Robot.m_joint[j].GetRevoluteJointState().m_rValue[0];
	//	qdot[j] = Robot.m_joint[j].GetRevoluteJointState().m_rValue[1];
	//	qddot[j] = Robot.m_joint[j].GetRevoluteJointState().m_rValue[2];
	//}
	//u = DynMatrices->M * (qddot_de - K0*(q - q_de) - K1*(qdot - qdot_de)) + DynMatrices->C * qdot + DynMatrices->N;
	//for (int j = 0; j < 7 ;j++)
	//{
	//	Robot.m_joint[j].m_State.m_rCommand = u[j];
	//}
	//
	////cout << i + 1 << "th joint actual torque: " << Robot.m_joint[i].GetRevoluteJointState().m_rValue[3] << endl;
	//cout << "joint actual torque: " << endl << (DynMatrices->M * qddot + DynMatrices->C * qdot + DynMatrices->N - u).norm() << endl;
	////cout << "DynMatrices->mMatrixG " << endl << DynMatrices->mMatrixG << endl;
	//cout << "joint error:" << (q-q_de).norm() << endl;
	//cout << "Lyapunov Function:" << 0.5 * r.transpose() * DynMatrices->M * r + (q-q_de).transpose() * Lambda_PBC*K_PBC * (q-q_de) << endl;
}

void User_CBFunc_Render(void* pvData)
{
	// you can draw whatever you want in OpenGL world
}

void User_CBFunc_KeyFunc(char key, void* pvData)
{
	switch(key) {
		case 'q':
		case 'Q':
			exit(0);
		default:
			break;
	}
}

// When we push the 'P' key, this function will work.
void User_CBFunc_Run_DYN()
{
    // Run dynamics simulation by setting glMainLoop idle function
    // as dynamics simulation step forward function.
    gViewer.SetLoopFunc(User_Simulation_Go_One_Step);
}

// When we push the 'O' key, this function will work.
void User_CBFunc_Pause_DYN()
{
    // Pause dynamics simulation by setting glMainLoop idle function
    // as empty function.
    gViewer.SetLoopFunc(User_Simulation_Pause);
}

// When we push the 'P' key, this function will be register to loop
// function of srSimpleViewer. Then the simulation will run.
void User_Simulation_Go_One_Step()
{
    // Dynamics simulation step forward function.
    gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
}

// When we push the 'O' key, this function will be register to loop
// function of srSimpleViewer. Then the simulation will be stopped.
void User_Simulation_Pause()
{
    // do nothings
}
