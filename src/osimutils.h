#include <string>
#include <OpenSim/OpenSim.h>

using namespace std;
using namespace SimTK;
using namespace OpenSim;


class OsimUtils
{
public:
	void static enableAllForces(State &state, Model &model);

	void static disableAllForces(State &state, Model &model);
};