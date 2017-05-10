//#include "OpenSim/OpenSim.h"
#include "OpenSim\Simulation\Manager\Manager.h"
#include "OpenSim/Simulation/InverseDynamicsSolver.h"
#include "osimutils.h"
#include "CustomAnalysis.h"

#include <vector>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

//void forwardSim(Model model);

// compute activation function for desirable knee angle
//void computeActivations (Model &model, const double angle,
    //vector<OpenSim::Function*> &controlFuncs, double &duration, bool store, Vector &ssai, Vector &ssaf, State &si);

// compute activation vector for current state (with static optimization)
//void calcSSact(Model &model, Vector &activations, State &si);

//class KneeController : public OpenSim::Controller
//{
//	OpenSim_DECLARE_CONCRETE_OBJECT(KneeController, Controller);
//
//    vector<double> timesLog;
//    vector<Vector> activationLog;
//	vector<OpenSim::Function*> cf;
//
//public:
//	vector<OpenSim::Function*> _activations;
//
//	KneeController(int numControls) : Controller()
//	{
//		setNumControls(numControls);
//	}
//
//	const vector<double> &getControlTimesLog() {
//		return timesLog;}
//
//    const vector<Vector> &getControlLog() {
//		return activationLog;}
//
//	void setControlFunctions( vector<OpenSim::Function*> &controlFuncs){
//		for (unsigned i=0; i< cf.size(); i++)
//            delete cf[i];
//
//        cf = controlFuncs;
//	}
//
//    void computeControls(const State &s, Vector &controls) const OVERRIDE_11
//	{
//		const double time = s.getTime();
//
//		for (unsigned i=0; i<cf.size(); ++i)
//			controls[i] = OsimUtils::evalFunc( cf[i], time);
//
//        // Log activations (Need to drop const-ness)
//        KneeController *this_ = const_cast<KneeController*>(this);
//        this_->timesLog.push_back(time);
//        this_->activationLog.push_back(controls);
//	}
//};

//void inverseSimulation(Model model);
//void staticOptimization(Model model);
void anteriorTibialLoadsFD(Model& model);
void monteCarloFD(Model& model, double random_dissipation, int j);
void forwardSimulation(Model& model);
void addTibialLoads(Model& model, double knee_angle);
void addFlexionController(Model& model);
void addExtensionController(Model& model);

void setKneeAngle(Model& model, SimTK::State &si, double angle_degrees);