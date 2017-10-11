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
AdaptiveControl AdaptiveController; 
<<<<<<< HEAD
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

=======
vector<Inertia*> vpEstimatedInertia; // 초기화 해줘야 함 Eigen::Matrix<double, 6, 6> I_tensor; I.ToArray(I_tensor.data());
DynamicsMatrix* DynMatrices;
//srSystem wam7_robot;

>>>>>>> 4a8c99ef818028b339baddf090ebe8c9f717b367
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
	gSpace.SetGravity(0.0, 0.0, -9.8);

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

<<<<<<< HEAD


=======
>>>>>>> 4a8c99ef818028b339baddf090ebe8c9f717b367
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	////- Space
	// Set simulation time step.
	gSpace.SetTimestep(0.0005);
	// Set gravity
<<<<<<< HEAD
	gSpace.SetGravity(0.0, 0.0, 9.8);
=======
	gSpace.SetGravity(0.0, 0.0, -9.8);
>>>>>>> 4a8c99ef818028b339baddf090ebe8c9f717b367
	// Set number of sub-step for rendering
	gSpace.SetNumberofSubstepForRendering(50);
}

// Simulation Setting.
void User_SimulationSetting()
{
<<<<<<< HEAD
	
=======
    // Set user control loop function.
    gSpace.SET_USER_CONTROL_FUNCTION(User_CBFunc_ControlLoop);

>>>>>>> 4a8c99ef818028b339baddf090ebe8c9f717b367
    // Initialize for dynamics simulation.
    gSpace.DYN_MODE_PRESTEP();
	DynMatrices = new DynamicsMatrix(&Robot, &AdaptiveController);

<<<<<<< HEAD
	///////////////////// erase here //////////////////////////

	I_list.push_back(&I1);
	I_list.push_back(&I2);
	I_list.push_back(&I3);
	I_list.push_back(&I4);
	I_list.push_back(&I5);
	I_list.push_back(&I6);
	I_list.push_back(&I7);

	DynMatrices->mvpEstimatedInertia = I_list;
	
	////////////////////////////////////////////

	// Set user control loop function.
	gSpace.SET_USER_CONTROL_FUNCTION(User_CBFunc_ControlLoop);

=======
>>>>>>> 4a8c99ef818028b339baddf090ebe8c9f717b367
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
	//AdaptiveControl.AdaptParameter();
	//AdaptiveControl.ApplyTorque(); // using updated inertia.
<<<<<<< HEAD
	
	
	// torque from dynamicMatrix
	DynMatrices->UpdateMatrices();
	int i = 3;
	VectorXd qddot = VectorXd::Zero(7);
	for (int j = 0; j < 7; j++)
	{
		qddot[j] = Robot.m_joint[j].GetRevoluteJointState().m_rValue[2];
		Robot.m_joint[j].m_State.m_rCommand = 0.01*I_list[j]->GetMass();
	}
	
	//cout << i + 1 << "th joint actual torque: " << Robot.m_joint[i].GetRevoluteJointState().m_rValue[3] << endl;
	cout << i+1 << "th joint actual torque: " << endl << (DynMatrices->M * qddot + DynMatrices->C * DynMatrices->qdot + DynMatrices->N) << endl;
	//cout << "DynMatrices->mMatrixG " << endl << DynMatrices->mMatrixG << endl;


=======
>>>>>>> 4a8c99ef818028b339baddf090ebe8c9f717b367
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
