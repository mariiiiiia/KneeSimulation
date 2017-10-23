#include "OpenSim\Simulation\Manager\Manager.h"
#include "OpenSim/Simulation/InverseDynamicsSolver.h"
#include "osimutils.h"
#include "CustomAnalysis.h"

#include <vector>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

/*
*	Perform a forward dynamic simulation of anterior tibial loads experiment
*	in 5 flexion angles: [0,15,30,60,90] degrees
*	{knee adduction is constraint}
*/
void anteriorTibialLoadsFD(Model& model, double knee_angle);
/*
*	Perform a forward dynamic simulation of active knee flexion experiment
*/
void flexionFDSimulation(Model& model);
/*
*	Perform a forward dynamic simulation of active knee flexion experiment and
*	visualize a dynamic hit map during this task
*/
void flexionFDSimulationWithHitMap(Model& model);


/*
*	Add external forces to tibia accordingly to knee angle 
*	so that the force is vertical to the tibia
*/
void addTibialLoads(Model& model, double knee_angle);
/*
*	Activate knee flexion muscles
*	setting a Constant Actuator Controller
*/
void addFlexionController(Model& model);
/*
*	Activate knee flexion muscles
*	setting a Constant Actuator Controller
*/
void addExtensionController(Model& model);

/* 
* set desired knee angle to the model, lock/unlock knee flexion coordinate, 
* unlock/constrai knee adduction to predefined rotations 
*/
void setKneeAngle(Model& model, SimTK::State &si, double angle_degrees, bool lock_knee_angle, bool lock_adduction);
// set hip angle to desired value (degrees) and lock this coordinate
void setHipAngle(Model& model, SimTK::State &si, double angle_degrees);
// set anterior tibial translation to predefined values according to knee angle
void setATT(Model& model, SimTK::State &si, double angle_degrees);
