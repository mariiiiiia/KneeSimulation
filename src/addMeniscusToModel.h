#include "OpenSim/OpenSim.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

/*	add meniscus bodies to knee and create joint
	bool LeftOrRight:	true for left body
 						false for right body
*/
void addMeniscusToKnee(Model& model, bool LeftOrRight);