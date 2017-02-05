#include "ACLsimulatorimpl.h"
#include "osimutils.h"
#include <ctime>
#include "CustomLigament.h"
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>


void forwardSim(Model model){
    model.setGravity(Vec3(0,-9.9,0));

	// name parameters needed
	Vector activs_initial, activs_final;		//	activations for the current position and activations for the next position
	vector<OpenSim::Function*> controlFuncs;	//	control functions for every t (time)
	vector<Vector> acts;						//	activations
	vector<double> actsTimes;					//	every t (=time) when I have an activation to apply
	double duration = 0.5;

	SimTK::State& state = model.initSystem();

	// compute muscle activations for specific knee angle (in rads)
	double knee_angle = -1.0;
	computeActivations(model, knee_angle, controlFuncs, duration, true, activs_initial, activs_final, state);

	// add controller to the model after adding the control functions to the controller
	KneeController *knee_controller = new KneeController( controlFuncs.size());
    knee_controller->setControlFunctions(controlFuncs);
	knee_controller->setActuators(model.getActuators());
	model.addController(knee_controller);

	// set model to initial state
	state = model.initSystem();
    //model.equilibrateMuscles(state);

    // Add reporters
    ForceReporter* forceReporter = new ForceReporter(&model);
    model.addAnalysis(forceReporter);

	// Simulate
    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0);
    manager.setFinalTime(2);

	time_t result = std::time(nullptr);
	std::cout << "\nBefore integrate(si) " << std::asctime(std::localtime(&result)) << endl;
	
    manager.integrate(state);

	result = std::time(nullptr);
	std::cout << "\nAfter integrate(si) " << std::asctime(std::localtime(&result)) << endl;

	// Save the simulation results
	Storage statesDegrees(manager.getStateStorage());
	statesDegrees.print("../outputs/kneeforwsim_states.sto");
	model.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
	statesDegrees.setWriteSIMMHeader(true);
	statesDegrees.print("../outputs/kneeforwsim_states_degrees.mot");

	// Get activation trajectory
    acts.resize(knee_controller->getControlLog().size());
    actsTimes.resize(knee_controller->getControlLog().size());

    vector<int> index(acts.size(), 0);
    for (int i = 0 ; i != index.size() ; i++) {
      index[i] = i;
    }

    sort(index.begin(), index.end(),
         [&](const int& a, const int& b) {
           return (knee_controller->getControlTimesLog()[a] <
                   knee_controller->getControlTimesLog()[b]);
         }
    );

    for (int i = 0 ; i != acts.size() ; i++) {
        acts[i] = knee_controller->getControlLog()[index[i]];
        actsTimes[i] = knee_controller->getControlTimesLog()[index[i]];
    }

    // Delete duplicates
    double ii = 0;
    for (unsigned i=1; i<acts.size(); i++) {
        if (actsTimes[i] <= actsTimes[ii]) {
            actsTimes.erase(actsTimes.begin()+i);
            acts.erase(acts.begin()+i);
            --i;
        }
        ii = i;
    }

	OsimUtils::writeFunctionsToFile( actsTimes, acts, "../outputs/force_Excitations_LOG.sto");
}

void computeActivations (Model &model, const double angle, vector<OpenSim::Function*> &controlFuncs,
			double &duration, bool store, Vector &so_activ_init, Vector &so_activ_final, State &si)
{
    calcSSact(model, so_activ_init, si);

	//set movement parameters
	const CoordinateSet &knee_r_cs = model.getJointSet().get("knee_r").getCoordinateSet();
    knee_r_cs.get("knee_angle_r").setValue(si, angle);
	model.equilibrateMuscles(si);

	calcSSact(model, so_activ_final, si);

    const int N = 9;

    const double t_eq = 0.000;
    const double dur_xc = (25.0 + 0.2 * angle) / 1000.;
    const double t_fix  = (99.0 + 0.5 * angle) / 1000.;

    double phases[N] = {
       -t_eq,
        0,
        0,
        dur_xc,
        dur_xc * 1.05,
        t_fix,
        t_fix + 0.005,
        t_fix + 0.010,
        t_fix*2
    };
    for (int i=0; i<N; i++) phases[i]+=t_eq;

    // Construct control functions
    controlFuncs.clear();
    double values[N];

    for (int i = 0; i < so_activ_final.size(); i++)  {
        double dssa = so_activ_final[i] - so_activ_init[i];
        int j=0;

        values[j++] = so_activ_init[i];
        values[j++] = values[j-1];

        values[j++] = so_activ_final[i] + dssa * 0.75;
        values[j++] = values[j-1];

        values[j++] = so_activ_init[i] + dssa * 0.975;
        values[j++] = values[j-1];

        values[j++] = so_activ_init[i] + dssa * 1.350;

        values[j++] = so_activ_final[i];
        values[j++] = values[j-1];

        OpenSim::Function *controlFunc = new PiecewiseLinearFunction(N, phases, values);
        controlFunc->setName(model.getActuators()[i].getName());
        controlFunc->setName("Excitation_" + model.getActuators()[i].getName());
        controlFuncs.push_back(controlFunc);
    }

    duration = t_eq + t_fix * 2;

	if (store == true) OsimUtils::writeFunctionsToFile( controlFuncs, "../outputs/_Excitations_LOG.sto", duration, 0.001 );
}

void calcSSact(Model &model, Vector &activations, State &si)
{
	// Perform a dummy forward simulation without forces,
    // just to obtain a state-series to be used by stat opt
    OsimUtils::disableAllForces(si, model);
	// Create the integrator and manager for the simulation.
    SimTK::RungeKuttaMersonIntegrator integrator( model.getMultibodySystem() );
    Manager manager( model, integrator );
	// Integrate from initial time to final time.
    manager.setInitialTime( 0);
    manager.setFinalTime( 2);
    std::cout << "\n\nIntegrating from 0 to 2 " << std::endl;
    manager.integrate( si);

    // Perform a quick static optimization that will give us
    // the steady state activations needed to overcome the passive forces
    OsimUtils::enableAllForces(si, model);

    Storage &states = manager.getStateStorage();
    states.setInDegrees(false);
    StaticOptimization so(&model);
    so.setStatesStore(states);
    State &s = model.initSystem();

    states.getData(0, s.getNY(), &s.updY()[0]);
    s.setTime(0);
    so.begin(s);
    so.end(s);

    Storage *as = so.getActivationStorage();
    int na = model.getActuators().getSize();
    activations.resize(na);
    int row = as->getSize() - 1;

    // Store activations to out vector
    for (int i = 0; i<na; i++)
	{
        as->getData(row, i, activations[i]);
		//// print results
		//cout << as->getColumnLabels()[i+1] << ": " << activations[i] << endl;
		//cout << as->getDataColumn() << "\n" << endl;
	}
}

void inverseSimulation(Model model)
{
	Array_<Real> times;
	double dur = 1.0;
	double step = 0.001;

	// Prepare time series
    InverseDynamicsSolver ids(model);
    times.resize((int)(dur / step) + 1, 0);
    for (unsigned i = 0; i < times.size(); i++)
        times[i] = step * i;

	// Solve for generalized joints forces
    State &s = model.initSystem();
	Vector ids_results = ids.solve(s, SimTK::Vector(0));

	for (unsigned i=0; i<ids_results.size(); i++)
		cout << i << " : " << ids_results(i) << endl;

	//for (int j=0; j<s.getQ().size(); j++)
	//{
	//	cout << "Q(" << j << "): " << s.getQ()[j] << endl;
	//}
	//
	//Array<std::string> stateNames = model.getStateVariableNames();    
	//printf("print state variable names (%d):\n", stateNames.size());
	//for (int j=0; j<stateNames.size(); j++)
 //   {
 //       printf("state variable name %d: %s\n", j, stateNames[j].c_str());
	//}
}

void staticOptimization(Model model)
{
    // set gravity off    
    //model.setGravity(Vec3(0,0,0));

    SimTK::State& state = model.initSystem();
    std::vector<std::vector<double>> acts;                // activations
    vector<double> actsTimes;                   // states.size()                      
    // Create the state sequence of motion (knee flexion)
    Storage states;
    states.setDescription("Knee flexion");
    Array<std::string> stateNames = model.getStateVariableNames();    
    stateNames.insert(0, "time");
    states.setColumnLabels(stateNames);
    Array<double> stateVals;

    const CoordinateSet &knee_r_cs = model.getJointSet().get("knee_r").getCoordinateSet();
    double knee_angle_r = -0.0290726;
    double t = 0.0;
    for (int i=0; i<20; i++)
    {
        knee_angle_r = -1.0/20 + knee_angle_r;
        knee_r_cs.get("knee_angle_r").setValue(state, knee_angle_r);
        model.getStateValues(state,stateVals);
        states.append( t, stateVals.size(), stateVals.get());
        t = t + 0.1;
    }
    t = t - 0.1;
    actsTimes.resize(states.getSize());
    states.setInDegrees(false);
    
	// Perform the static optimization
    StaticOptimization so(&model);
    so.setStatesStore(states);
    int ns = states.getSize(); 
    double ti, tf;
    states.getTime(0, ti);
    states.getTime(ns-1, tf);
    so.setStartTime(ti);
    so.setEndTime(tf);

    // Run the analysis loop
    state = model.initSystem();
    for (int i = 0; i < ns; i++) {
        states.getData(i, state.getNY(), &state.updY()[0]);
        Real t = 0.0;
        states.getTime(i, t);
        state.setTime(t);
        model.assemble(state);
        model.getMultibodySystem().realize(state, SimTK::Stage::Velocity);

        if (i == 0 )
          so.begin(state);
        else if (i == ns)
          so.end(state);
        else
          so.step(state, i);
    }

	// store .mot file
	Storage* activations = so.getActivationStorage();
	activations->print("../outputs/so_acts.sto");
	Storage* forces = so.getForceStorage();
	forces->print("../outputs/so_forces.sto");

	//cout << forces->getName() << endl;
}

void forwardSimulation(Model& model)
{
	// Get a reference to the model's ground body
	//OpenSim::Body& ground = model.getGroundBody();
	// Add display geometry to the ground to visualize in the GUI
	//ground.addDisplayGeometry("ground.vtp");

	// add external force
	//addExternalForce(model, -0.3);
	//addExternalForce(model, 0.1);   

	addFlexionController(model);
	//addExtensionController(model);

	// init system
	std::time_t result = std::time(nullptr);
	std::cout << "\nBefore initSystem() " << std::asctime(std::localtime(&result)) << endl;
	SimTK::State& si = model.initSystem();
	result = std::time(nullptr);
	std::cout << "\nAfter initSystem() " << std::asctime(std::localtime(&result)) << endl;
	
	// set gravity
	//model.updGravityForce().setGravityVector(si, Vec3(-9.80665,0,0));
	model.updGravityForce().setGravityVector(si, Vec3(0,0,0));
	
	// disable muscles
	string muscle_name;
	for (int i=0; i<model.getActuators().getSize(); i++)
	{
		muscle_name = model.getActuators().get(i).getName();

		model.getActuators().get(i).setDisabled(si, true);

		if (muscle_name == "bifemlh_r" || muscle_name == "bifemsh_r" || muscle_name == "grac_r" \
			|| muscle_name == "lat_gas_r" || muscle_name == "med_gas_r" || muscle_name == "sar_r" \
			|| muscle_name == "semimem_r" || muscle_name == "semiten_r" \
			|| muscle_name == "rect_fem_r" || muscle_name == "vas_med_r" || muscle_name == "vas_int_r" || muscle_name == "vas_lat_r" )
				model.getActuators().get(i).setDisabled(si, false);
	}

	// set knee angles  
	//const CoordinateSet &knee_r_cs = model.getJointSet().get("knee_r").getCoordinateSet();
	//knee_r_cs.get("knee_angle_r").setValue(si, -2.09439510);  // -120 degrees
	//knee_r_cs.get("knee_angle_r").setValue(si, -1.74532925);  // -100 degrees
	//knee_r_cs.get("knee_angle_r").setValue(si, -1.570796326);  // -90 degrees
	//knee_r_cs.get("knee_angle_r").setValue(si, -1.39626340);  // -80 degrees
	//knee_r_cs.get("knee_angle_r").setValue(si, -1.04719755);  // -60 degrees
	//knee_r_cs.get("knee_angle_r").setValue(si, -0.6981317008);  // -40 degrees
	//knee_r_cs.get("knee_angle_r").setValue(si, -0.34906585);  // -20 degrees
    //knee_r_cs.get("knee_angle_r").setValue(si, -0.029); 
	//knee_r_cs.get("knee_angle_r").setLocked(si, true);

	//knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.02025397);
	//knee_r_cs.get("knee_inferior_superior_r").setValue(si, -0.38544807);

	// set initial activations
	//const Array<Object*> flexors = model.getForceSet().getGroup( "R_knee_bend")->getMembers();
	//Thelen2003Muscle* flex_muscle;
	//for (int i=0; i<flexors.size(); i++)
	//{
	//	flex_muscle = static_cast<Thelen2003Muscle*> (flexors.get(i));
	//	flex_muscle->setActivation( si, 1.0);
		//flex_muscle->set_deactivation_time_constant( 0.5);
	//}

	model.equilibrateMuscles( si);

	// Add reporters
    ForceReporter* forceReporter = new ForceReporter(&model);
    model.addAnalysis(forceReporter);

	CustomAnalysis* customReporter = new CustomAnalysis(&model, "r");
	model.addAnalysis(customReporter);

	// Create the integrator and manager for the simulation.
	SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
	//integrator.setAccuracy(1.0e-3);
	//integrator.setFixedStepSize(0.001);
	Manager manager(model, integrator);

	// Define the initial and final simulation times
	double initialTime = 0.0;
	double finalTime = 0.3;

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	std::cout<<"\n\nIntegrating from "<<initialTime<<" to " <<finalTime<<std::endl;

	result = std::time(nullptr);
	std::cout << "\nBefore integrate(si) " << std::asctime(std::localtime(&result)) << endl;

	manager.integrate(si);

	result = std::time(nullptr);
	std::cout << "\nAfter integrate(si) " << std::asctime(std::localtime(&result)) << endl;

	// Save the simulation results
	Storage statesDegrees(manager.getStateStorage());
	statesDegrees.print("../outputs/states_flex.sto");
	model.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
	statesDegrees.setWriteSIMMHeader(true);
	statesDegrees.print("../outputs/states_degrees_flex.mot");
	// force reporter results
	forceReporter->getForceStorage().print("../outputs/force_reporter_flex.mot");
	customReporter->print( "../outputs/custom_reporter_flex.mot");
}

void addExternalForce(Model& model, double const_point_y)
{
	// Specify properties of a force function to be applied to the block
	double timeX[2] = {0.1, 0.3}; // time nodes for linear function
	double timeY[2] = {0.0, 0.3}; // time nodes for linear function
	double fXofT[2] = {100, 100}; // force values at t1 and t2
	double fYofT[2] = {250, 0}; // force values at t1 and t2
	//double pYofT[2] = {0, 0.1}; // point in x values at t1 and t2
  
	// Create a new linear functions for the force and point components
	PiecewiseLinearFunction *forceX = new PiecewiseLinearFunction(2, timeX, fXofT);
	PiecewiseLinearFunction *forceY = new PiecewiseLinearFunction(2, timeY, fYofT);
	//PiecewiseLinearFunction *pointY = new PiecewiseLinearFunction(2, time, pYofT);
  
	// Create a new prescribed force applied to the block
	PrescribedForce *prescribedForce = new PrescribedForce( &model.updBodySet().get("tibia_upper_r"));
	prescribedForce->setName("prescribedForce");
  
	// Set the force and point functions for the new prescribed force
	prescribedForce->setForceFunctions( new Constant(0), new Constant(800.0), new Constant(0.0));
	prescribedForce->setPointFunctions(new Constant(0.0), new Constant(0), new Constant(0.0));

	// Add the new prescribed force to the model
	model.addForce(prescribedForce);
}

void addFlexionController(Model& model)
{
	PrescribedController* controller = new PrescribedController();
	controller->setName( "flexion_controller");
	controller->setActuators( model.updActuators());
	
	double control_time[3] = {0, 0.1, 0.2}; // time nodes for linear function
	double control_acts[3] = {0, 0.2, 0.2}; // force values at t1 and t2
	//control_func->setName( "constant_control_func");

	string muscle_name;
	for (int i=0; i<model.getActuators().getSize(); i++)
	{
		muscle_name = model.getActuators().get(i).getName();
		// hamstrings: bi*, semi*
		if ( muscle_name == "bifemlh_r" || muscle_name == "bifemsh_r" || muscle_name == "grac_r" \
			|| muscle_name == "lat_gas_r" || muscle_name == "med_gas_r" || muscle_name == "sar_r" \
			|| muscle_name == "semimem_r" || muscle_name == "semiten_r")
		{
			Constant* ccf = new Constant(1.0);
			//PiecewiseLinearFunction *ccf = new PiecewiseLinearFunction( 3, control_time, control_acts);
			controller->prescribeControlForActuator( i, ccf);
		}
		else 
		{
			Constant* zccf = new Constant(0);
			controller->prescribeControlForActuator( i, zccf);
		}
	}
	model.addController( controller);
}

void addExtensionController(Model& model)
{
	PrescribedController* controller = new PrescribedController();
	controller->setName( "extension_controller");
	controller->setActuators( model.updActuators());
	
	//double control_time[2] = {0, 0.2}; // time nodes for linear function
	//double control_acts[2] = {0.6, 1.0}; // force values at t1 and t2
	//control_func->setName( "constant_control_func");

	string muscle_name;
	for (int i=0; i<model.getActuators().getSize(); i++)
	{
		muscle_name = model.getActuators().get(i).getName();
		// activate quadriceps
		if (muscle_name == "rect_fem_r" || muscle_name == "vas_med_r" || muscle_name == "vas_int_r" || muscle_name == "vas_lat_r" )
		{
			Constant* ccf = new Constant(0.5);
			//PiecewiseLinearFunction *ccf = new PiecewiseLinearFunction( 2, control_time, control_acts);
			controller->prescribeControlForActuator( i, ccf);
		}
		else 
		{
			Constant* zccf = new Constant(0);
			controller->prescribeControlForActuator( i, zccf);
		}
	}
	model.addController( controller);
}
