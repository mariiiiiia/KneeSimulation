#include "OpenSim/OpenSim.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

/*
*	Add knee contact geometries
*
*	bool left_knee:	true for Left body 
*						false for Right body
*/
void addKneeContactGeometries(Model& model, bool left_knee);

/*
*	Add a contact mesh from obj file <objName> to body <bodyname>
*/
void addContactGeometry(Model& model, string bodyName, string objName);

/*
*	Add Elastic Foundation Forces to knee contact geometries 
*
*	men_***: meniscus EFF parameter values
*	art_***: articular cartilage EFF parameter values
*	bool left_knee:	true for Left body 
*						false for Right body
*/
void addEFForces(Model& model, double men_stiff, double men_diss, double men_us, double men_ud, double men_uv,
	double art_stiff, double art_diss, double art_us, double art_ud, double art_uv, bool left_knee);
