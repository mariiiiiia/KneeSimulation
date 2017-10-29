//#include "OpenSim/OpenSim.h"
#include "OpenSim\Simulation\Model\Model.h"
#include "OpenSim\Simulation\Model\BodySet.h"
#include "OpenSim\Simulation\SimbodyEngine\WeldJoint.h"
#include "OpenSim\Simulation\SimbodyEngine\FreeJoint.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

/*	
*	Add meniscii bodies to model and 
*	join to tibia with Weld Joints
*
*	bool left_knee:	true for left body
* 						false for right body
*/
void addMeniscusWeldJoints(Model& model, bool left_knee);

/*	
*	Add lower femur bodies to model and 
*	join to upper femur with Weld Joints
*
*	bool left_knee:	true for left body
* 						false for right body
*/
void addFemurWeldJoints(Model& model, bool left_knee);

/*	
*	Add upper tibia bodies to model and 
*	join to lower tibia with Weld Joints
*
*	bool left_knee:	true for left body
* 						false for right body
*/
void addTibiaWeldJoints(Model& model, bool left_knee);

