//#include "OpenSim/OpenSim.h"
#include "OpenSim\Simulation\Model\Model.h"
#include "OpenSim\Simulation\Model\BodySet.h"
#include "OpenSim\Simulation\SimbodyEngine\WeldJoint.h"
#include "OpenSim\Simulation\SimbodyEngine\FreeJoint.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

/*	add meniscus bodies to knee and create joint
	bool LeftOrRight:	true for left body
 						false for right body
*/
void addMeniscusWeldJoints(Model& model, bool LeftOrRight);

void addFemurWeldJoints(Model& model, bool LeftOrRight);

void addTibiaWeldJoints(Model& model, bool LeftOrRight);

void addUpperTibiaFreeJoints(Model& model, bool LeftOrRight);