#include "OpenSim\OpenSim.h"
//#include "OpenSim\Simulation\Control\PrescribedController.h"

#include <string>
#include <vector>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void forwardsim(Model model, SimTK::State& si);

class KneeController : public OpenSim::Controller 
{
	OpenSim_DECLARE_CONCRETE_OBJECT(KneeController, Controller);

public:
	KneeController() : Controller()
	{
		setNumControls(9);
	}

    void computeControls(const State &s, Vector &controls) const OVERRIDE_11
	{
		// Get the current time in the simulation.
		double t = s.getTime();

		// Get pointers to each of the muscles in the model.
		const Millard2012EquilibriumMuscle* muscle1 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("BIFEMLH"));
		const Millard2012EquilibriumMuscle* muscle2 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("BIFEMSH"));
		const Millard2012EquilibriumMuscle* muscle3 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("GRA"));
		const Millard2012EquilibriumMuscle* muscle4 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("HAMSTRINGS"));
		const Millard2012EquilibriumMuscle* muscle5 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("LATGAS"));
		const Millard2012EquilibriumMuscle* muscle6 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("MEDGAS"));
		const Millard2012EquilibriumMuscle* muscle7 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("SAR"));
		const Millard2012EquilibriumMuscle* muscle8 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("SEMIMEM"));
		const Millard2012EquilibriumMuscle* muscle9 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("SEMITEN"));



		// Thelen muscle has only one control
	    Vector muscleControl(1, 23.321);

		// Add in the controls computed for this muscle to the set of all model controls
		muscle1->addInControls(muscleControl, controls);
		muscle2->addInControls(muscleControl, controls);
		muscle3->addInControls(muscleControl, controls);
		muscle4->addInControls(muscleControl, controls);
		muscle5->addInControls(muscleControl, controls);
		muscle6->addInControls(muscleControl, controls);
		muscle7->addInControls(muscleControl, controls);
		muscle8->addInControls(muscleControl, controls);
		muscle9->addInControls(muscleControl, controls);

	}
};