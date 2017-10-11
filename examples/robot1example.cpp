// Include necessary header files.
#include "Renderer/SimpleViewer.h"    // for rendering
#include "srDyn/srSpace.h"            // for dynamics

// If you want to use some predefined rendering functions,
// include below.
#include "srg/srgGeometryDraw.h"		// for User rendering

// Ground
#include "Ground.h"
#include "robot1.h"

// Get srSimpleViewer instance. (singleton)
srSimpleViewer& gViewer = srSimpleViewer::GetInstance();

// Declare srSpace instance.
srSpace gSpace;

// >>>>> DECLARE VARIABLES HERE. <<<<<
// Ground
Ground gGround;

robot1* firstRobot;
robot1* secondRobot;

srIRSensor gIRSensor;


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

void User_CBFunc_GoForward();
void User_CBFunc_GoBackward();
void User_CBFunc_Stop();

// If you want to control your mobile robot, we need a contorl function.
// And this function must be registered at srSpace.
void User_CBFunc_ControlLoop();    // User control loop.

								   // Callback function for user rendering
								   //   put your code to render what you want. (OpenGL functions avaliable.)
void User_CBFunc_Render(void* pvData);

// Callback function for user key binding
void User_CBFunc_Exit(char key, void* pvData);

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

	// STEP 4: Run window view.
	gViewer.Run();

	return 0;
}

void User_Modeling()
{
	gSpace.AddSystem(gGround.BuildGround());

	// >>>>> WRITE YOUR MODELING CODE HERE. >>>>>

	firstRobot = new robot1;
	secondRobot = new robot1;
	firstRobot->buildRobot1(SE3(Vec3(0,0,-5)));
	secondRobot->buildRobot1(EulerZYX(Vec3(0,SR_PI,0), Vec3(0,0,5)));
	gSpace.AddSystem(firstRobot);
	gSpace.AddSystem(secondRobot);

	//firstRobot->link[2].SetFriction(0.0);

	gIRSensor.SetRange(1.0, 0.1);
	firstRobot->link[3].AddSensor(&gIRSensor);
	gIRSensor.SetLocalFrame(EulerZYX(Vec3(0, -0.5*SR_PI, 0.0), Vec3(0, 0.0, 1.5)));
	gIRSensor.Detect();

	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	////- Space
	// Set simulation time step.
	gSpace.SetTimestep(0.001);
	// Set gravity
	gSpace.SetGravity(0.0, 0.0, -9.8);
	// Set number of sub-step for rendering
	gSpace.SetNumberofSubstepForRendering(50);
}

// Simulation Setting.
void User_SimulationSetting()
{
	// Set user control loop function.
	gSpace.SET_USER_CONTROL_FUNCTION(User_CBFunc_ControlLoop);

	// Initialize for dynamics simulation.
	gSpace.DYN_MODE_PRESTEP();

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

	gViewer.SetUserKeyFunc(User_CBFunc_Exit, NULL);

	gViewer.SetKeyFunc(User_CBFunc_GoForward, 'W');
	gViewer.SetKeyFunc(User_CBFunc_GoForward, 'w');

	gViewer.SetKeyFunc(User_CBFunc_Stop, 'S');
	gViewer.SetKeyFunc(User_CBFunc_Stop, 's');

	gViewer.SetKeyFunc(User_CBFunc_GoBackward, 'X');
	gViewer.SetKeyFunc(User_CBFunc_GoBackward, 'x');
}

// >>>>> WRITE YOUR CONTROL CODE HERE. <<<<<
void User_CBFunc_ControlLoop()
{

	//////		IR sensor feedback			////

	//real detectedValue = gIRSensor.GetDetectedValue();
	//real sensorMax = gIRSensor.GetMaxRange();
	//real kp = 3;

	//if (sensorMax>detectedValue)
	//{
	//	//right wheel
	//	firstRobot->wheelJoint[0].m_State.m_rCommand = -kp * (sensorMax - detectedValue);
	//	//left  wheel
	//	firstRobot->wheelJoint[1].m_State.m_rCommand = -kp * (sensorMax - detectedValue);
	//}

}

void User_CBFunc_Render(void* pvData)
{
	// you can draw whatever you want in OpenGL world
}

void User_CBFunc_Exit(char key, void* pvData)
{
	switch (key) {
	case 'q':
	case 'Q':
		exit(0);
	default:
		break;
	}
}



void User_CBFunc_GoForward()
{
	//right wheel
	firstRobot->wheelJoint[0].m_State.m_rCommand = 1.0;
	//left  wheel
	firstRobot->wheelJoint[1].m_State.m_rCommand = 1.0;
}
void User_CBFunc_GoBackward()
{
	//right wheel
	firstRobot->wheelJoint[0].m_State.m_rCommand = -1.0;
	//left  wheel
	firstRobot->wheelJoint[1].m_State.m_rCommand = -1.0;
}
void User_CBFunc_TurnRight()
{
	//right wheel
	firstRobot->wheelJoint[0].m_State.m_rCommand = -1.0;
	//left  wheel
	firstRobot->wheelJoint[1].m_State.m_rCommand = 1.0;
}
void User_CBFunc_TurnLeft()
{
	//right wheel
	firstRobot->wheelJoint[0].m_State.m_rCommand = 1.0;
	//left  wheel
	firstRobot->wheelJoint[1].m_State.m_rCommand = -1.0;
}
void User_CBFunc_Stop()
{
	//right wheel
	firstRobot->wheelJoint[0].m_State.m_rCommand = 0.0;
	//left  wheel
	firstRobot->wheelJoint[1].m_State.m_rCommand = 0.0;
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
