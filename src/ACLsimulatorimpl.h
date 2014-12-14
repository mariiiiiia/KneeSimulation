#include "OpenSim\OpenSim.h"
#include "osimutils.h"

//#include <string>
#include <vector>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void forwardsim(Model model, SimTK::State& si);

void computeActivations (Model &model, const double angle,
    vector<OpenSim::Function*> &controlFuncs, double &duration, bool store, Vector &ssai, Vector &ssaf, State &si);

void calcSSact(Model &model, Vector &activations, State &si);

class KneeController : public OpenSim::Controller 
{
	OpenSim_DECLARE_CONCRETE_OBJECT(KneeController, Controller);

    vector<double> timesLog;
    vector<Vector> activationLog;

public:
	Vector _activations;

	KneeController(int numControls) : Controller()
	{
		setNumControls(numControls);
	}

	const vector<double> &getControlTimesLog() {return timesLog;}

    const vector<Vector> &getControlLog() {return activationLog;}


    void computeControls(const State &s, Vector &controls) const OVERRIDE_11
	{
		for (int i=0; i<_activations.size(); ++i)
            controls[i] = _activations[i];

        // Log activations (Need to drop const-ness)
        KneeController *this_ = const_cast<KneeController*>(this);
        this_->timesLog.push_back(s.getTime());
        this_->activationLog.push_back(_activations);
	}

 //   void computeControls(const State &s, Vector &controls) const OVERRIDE_11
	//{
	//	// Get the current time in the simulation.
	//	double t = s.getTime();

	//	// Get pointers to each of the muscles in the model.
	//	const Millard2012EquilibriumMuscle* muscle1 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("BIFEMLH"));
	//	const Millard2012EquilibriumMuscle* muscle2 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("BIFEMSH"));
	//	const Millard2012EquilibriumMuscle* muscle3 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("GRA"));
	//	const Millard2012EquilibriumMuscle* muscle4 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("HAMSTRINGS"));
	//	const Millard2012EquilibriumMuscle* muscle5 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("LATGAS"));
	//	const Millard2012EquilibriumMuscle* muscle6 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("MEDGAS"));
	//	const Millard2012EquilibriumMuscle* muscle7 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("SAR"));
	//	const Millard2012EquilibriumMuscle* muscle8 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("SEMIMEM"));
	//	const Millard2012EquilibriumMuscle* muscle9 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("SEMITEN"));

	//	// Thelen muscle has only one control
	//    Vector sar_gra_Control(1, 5.034);
	//	Vector semiten_Control(1, 16.500);
	//	Vector bifemsh_Control(1, 20.095);
	//	Vector latgas_Control(1, 21.700);
	//	Vector semimem_Control(1, 12.1700);
	//	Vector hamstings_Control(1, 30.665);
	//	Vector medgas_Control(1, 47.574);
	//	Vector bifemlh_Control(1, 30.665);

	//	// Add in the controls computed for this muscle to the set of all model controls
	//	muscle1->addInControls(bifemlh_Control, controls);
	//	muscle2->addInControls(bifemsh_Control, controls);
	//	muscle3->addInControls(sar_gra_Control, controls);
	//	muscle4->addInControls(hamstings_Control, controls);
	//	muscle5->addInControls(latgas_Control, controls);
	//	muscle6->addInControls(medgas_Control, controls);
	//	muscle7->addInControls(sar_gra_Control, controls);
	//	muscle8->addInControls(semimem_Control, controls);
	//	muscle9->addInControls(semiten_Control, controls);

	//}
};
