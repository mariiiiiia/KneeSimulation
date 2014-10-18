#include "ACLsimulator.h"


void forwardsim(Model model, SimTK::State& si);

class KneeController : public OpenSim::Controller 
{
	OpenSim_DECLARE_CONCRETE_OBJECT(KneeController, Controller);

public:
	KneeController() : Controller()
	{
		setNumControls(1);
	}

    void computeControls(const State &s, Vector &controls) const OVERRIDE_11
	{
		// Get the current time in the simulation.
		//double t = s.getTime();
		// Get pointers to each of the muscles in the model.
		const Actuator &act = static_cast<const Actuator&> (getActuatorSet().get("HAMSTRINGS"));
		const Millard2012EquilibriumMuscle* muscle1 = static_cast<const Millard2012EquilibriumMuscle*>(&getActuatorSet().get("HAMSTRINGS"));

		// Thelen muscle has only one control
	    Vector muscleControl(1, 6.0);

		// Add in the controls computed for this muscle to the set of all model controls
		muscle1->addInControls(muscleControl, controls);
	}
};