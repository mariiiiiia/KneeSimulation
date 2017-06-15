//#include "OpenSim/OpenSim.h"
#include "OpenSim\Simulation\Manager\Manager.h"
#include "OpenSim/Simulation/InverseDynamicsSolver.h"
#include "osimutils.h"
#include "CustomAnalysis.h"

#include <vector>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void anteriorTibialLoadsFD(Model& model);
void monteCarloFD(Model& model, double random_dissipation, int j);
void flexionFDSimulation(Model& model);
void flexionFDSimulationWithHitMap(Model& model);
void addTibialLoads(Model& model, double knee_angle);
void addFlexionController(Model& model);
void addExtensionController(Model& model);

void setKneeAngle(Model& model, SimTK::State &si, double angle_degrees, bool lock_knee_angle);
void setHipAngle(Model& model, SimTK::State &si, double angle_degrees);