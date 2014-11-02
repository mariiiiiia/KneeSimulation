#include "ACLsimulatorimpl.h"

void forwardsim(Model model, SimTK::State& si){
	double initialTime = 0.0;
	double finalTime = 1.0;

	//Vector controls;

	//// Create the controller.
	//KneeController *controller = new KneeController();
	//controller->setActuators( model.getActuators());
	//
 //   // Add the controller to the Model.
 //   model.addController( controller );

	//const Millard2012EquilibriumMuscle* muscle1 = static_cast<const Millard2012EquilibriumMuscle*>(&model.getActuators().get("BIFEMLH"));
	//const Millard2012EquilibriumMuscle* muscle2 = static_cast<const Millard2012EquilibriumMuscle*>(&model.getActuators().get("BIFEMSH"));
	//const Millard2012EquilibriumMuscle* muscle3 = static_cast<const Millard2012EquilibriumMuscle*>(&model.getActuators().get("GRA"));
	//const Millard2012EquilibriumMuscle* muscle4 = static_cast<const Millard2012EquilibriumMuscle*>(&model.getActuators().get("HAMSTRINGS"));
	//const Millard2012EquilibriumMuscle* muscle5 = static_cast<const Millard2012EquilibriumMuscle*>(&model.getActuators().get("LATGAS"));
	//const Millard2012EquilibriumMuscle* muscle6 = static_cast<const Millard2012EquilibriumMuscle*>(&model.getActuators().get("MEDGAS"));
	//const Millard2012EquilibriumMuscle* muscle7 = static_cast<const Millard2012EquilibriumMuscle*>(&model.getActuators().get("SAR"));
	//const Millard2012EquilibriumMuscle* muscle8 = static_cast<const Millard2012EquilibriumMuscle*>(&model.getActuators().get("SEMIMEM"));
	//const Millard2012EquilibriumMuscle* muscle9 = static_cast<const Millard2012EquilibriumMuscle*>(&model.getActuators().get("SEMITEN"));

	
	//muscle1->setFiberLength(si, 0.072);
	//muscle2->setFiberLength(si, 0.155);
	//muscle3->setFiberLength(si, 0.294);
	//muscle4->setFiberLength(si, 0.072);
	//muscle5->setFiberLength(si, 0.048);
	//muscle6->setFiberLength(si, 0.031);
	//muscle7->setFiberLength(si, 0.523);
	//muscle8->setFiberLength(si, 0.036);
	//muscle9->setFiberLength(si, 0.169);

	// Create a prescribed controller that simply applies controls as function of time
	PrescribedController *muscleController = new PrescribedController();
	muscleController->setActuators(model.getActuators());

	// Define linear functions for the control values for the two muscles
	Array<double> slopeAndIntercept1(0.0, 2); // array of 2 doubles

	// muscle1 control has slope of -1 starting 1 at t = 0
	slopeAndIntercept1[0] = 1.0/(finalTime-initialTime); 
	slopeAndIntercept1[1] = 1.0;

	// Set the indiviudal muscle control functions for the prescribed muscle controller
	muscleController->prescribeControlForActuator("BIFEMLH", new LinearFunction(slopeAndIntercept1));
	muscleController->prescribeControlForActuator("BIFEMSH", new LinearFunction(slopeAndIntercept1)); 
	muscleController->prescribeControlForActuator("GRA", new LinearFunction(slopeAndIntercept1)); 
	muscleController->prescribeControlForActuator("HAMSTRINGS", new LinearFunction(slopeAndIntercept1)); 
	muscleController->prescribeControlForActuator("LATGAS", new LinearFunction(slopeAndIntercept1)); 
	muscleController->prescribeControlForActuator("MEDGAS", new LinearFunction(slopeAndIntercept1)); 
	muscleController->prescribeControlForActuator("SAR", new LinearFunction(slopeAndIntercept1)); 
	muscleController->prescribeControlForActuator("SEMIMEM", new LinearFunction(slopeAndIntercept1)); 
	muscleController->prescribeControlForActuator("SEMITEN", new LinearFunction(slopeAndIntercept1)); 
	
	// Add the muscle controller to the model
	//model.addController(muscleController);

	si = model.initSystem();
    //model.equilibrateMuscles(si);

	// Create the integrator and manager for the simulation.
    SimTK::RungeKuttaMersonIntegrator integrator( model.getMultibodySystem() );
    Manager manager( model, integrator );
	// Integrate from initial time to final time.
    manager.setInitialTime( initialTime);
    manager.setFinalTime( finalTime );
    std::cout << "\n\nIntegrating from " << initialTime << " to " << finalTime << std::endl;
    manager.integrate( si );
	
	// Save the simulation results
	Storage statesDegrees(manager.getStateStorage());
	statesDegrees.print("kneeforwsim_states.sto");
	model.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
	statesDegrees.setWriteSIMMHeader(true);
	statesDegrees.print("kneeforwsim_states_degrees.mot");
}
