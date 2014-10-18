#include "ACLsimulatorimpl.h"


void forwardsim(Model model, SimTK::State& si){
	//Vector controls;

	// Create the controller.
    KneeController *controller = new KneeController();
	controller->setActuators( model.getActuators());
   
    // Add the controller to the Model.
    model.addController( controller );

	si = model.initSystem(); 
	// Create the integrator and manager for the simulation.
    SimTK::RungeKuttaMersonIntegrator integrator( model.getMultibodySystem() );
    Manager manager( model, integrator );
	double initialTime = 0.0;
	double finalTime = 4.0;
	// Integrate from initial time to final time.
    manager.setInitialTime( initialTime);
    manager.setFinalTime( finalTime );
    std::cout << "\n\nIntegrating from " << "0" << " to " << "4.0" << std::endl;
    manager.integrate( si );
	
	// Save the simulation results
	Storage statesDegrees(manager.getStateStorage());
	statesDegrees.print("kneeforwsim_states.sto");
	model.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
	statesDegrees.setWriteSIMMHeader(true);
	statesDegrees.print("kneeforwsim_states_degrees.mot");
}
