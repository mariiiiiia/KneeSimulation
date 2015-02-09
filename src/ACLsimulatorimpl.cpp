#include "ACLsimulatorimpl.h"
#include "osimutils.h"

void forwardSim(Model model, SimTK::State& si){
	// name parameters needed
	Vector activs_initial, activs_final;		//	activations for the current position and activations for the next position
	vector<OpenSim::Function*> controlFuncs;	//	control functions for every t (time)
	vector<Vector> acts;						//	activations
	vector<double> actsTimes;					//	every t (=time) when I have an activation to apply
	double initialTime = 0.0;					
	double finalTime = 2.0;
	double duration = 0.5;

	si = model.initSystem();

	// compute muscle activations for specific knee angle (in rads)
	double knee_angle = -1.0; 
	computeActivations(model, knee_angle, controlFuncs, duration, true, activs_initial, activs_final, si);

	// add controller to the model after adding the control functions to the controller
	KneeController *knee_controller = new KneeController( model.getActuators().getSize());
    knee_controller->setControlFunctions(controlFuncs);
	knee_controller->setActuators(model.getActuators());
	//knee_controller->connectToModel(model);      // I don't know if it's needed
	model.addController(knee_controller);
	 
	// set model to initial state
	si = model.initSystem();
    //model.equilibrateMuscles(si);

    // Add reporters
    ForceReporter* forceReporter = new ForceReporter(&model);
    model.addAnalysis(forceReporter);

	// Simulate
    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0);
    manager.setFinalTime(duration);
    manager.integrate(si);

	// Save the simulation results
	Storage statesDegrees(manager.getStateStorage());
	statesDegrees.print("kneeforwsim_states.sto");
	model.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
	statesDegrees.setWriteSIMMHeader(true);
	statesDegrees.print("kneeforwsim_states_degrees.mot");

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
    for (int i=1; i<acts.size(); i++) {
        if (actsTimes[i] <= actsTimes[ii]) {
            actsTimes.erase(actsTimes.begin()+i);
            acts.erase(acts.begin()+i);
            --i;
        }
        ii = i;
    }

	OsimUtils::writeFunctionsToFile( actsTimes, acts, "force_Excitations_LOG.sto");
}

void computeActivations (Model &model, const double angle, vector<OpenSim::Function*> &controlFuncs, 
			double &duration, bool store, Vector &so_activ_init, Vector &so_activ_final, State &si)
{
	calcSSact(model, so_activ_init, si);
	
	//set movement parameters
	const CustomJoint &knee_r_joint = static_cast<const CustomJoint&>(model.getJointSet().get("knee_r"));
    knee_r_joint.get_CoordinateSet().get("knee_angle_r").setValue(si, angle);
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

	if (store == true) OsimUtils::writeFunctionsToFile( controlFuncs, "_Excitations_LOG.sto", duration, 0.001 );
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
    manager.setFinalTime( 2 );
    std::cout << "\n\nIntegrating from 0 to 2 " << std::endl;
    manager.integrate( si );

    // Perform a quick static optimization that will give us
    // the steady state activations needed to overcome the passive forces
    OsimUtils::enableAllForces(si, model);
	//// initiate all muscle activations at 0.005 (DON'T KNOW WHY)
	//const Set<Muscle> &muscleSet = model.getMuscles();
	//   for(int i=0; i< muscleSet.getSize(); i++ ){
	//		muscleSet[i].setActivation(si, 0.05);
	//	}
    Storage &states = manager.getStateStorage();
    states.setInDegrees(false);
    StaticOptimization so(&model);

    so.setStatesStore(states);
    si = model.initSystem();
    states.getData(0, si.getNY(), &si.updY()[0]);
    si.setTime(0);
    so.begin(si);
    so.end(si);

    Storage *as = so.getActivationStorage();
    int na = model.getActuators().getSize();
    activations.resize(na);
    int row = as->getSize() - 1;
	
    // Store activations to out vector
    for (int i = 0; i<na; i++)
	{
        as->getData(row, i, activations[i]);
		cout << as->getColumnLabels()[i+1] << ": " << activations[i] << endl;
		//cout << as->getDataColumn() << "\n" << endl;
	}
}

// for any post XML deseraialization intialization
void KneeController::connectToModel(Model& model)
{
	Super::connectToModel(model);

	if(getProperty_actuator_list().size() > 0){
		if(IO::Uppercase(get_actuator_list(0)) == "ALL"){
			setActuators(model.getActuators());
			// setup actuators to ensure actuators added by controllers are also setup properly
			// TODO: Adopt the controls (discrete state variables) of the Actuator
			return;
		}
		else{
			Set<Actuator> actuatorsByName;
			for(int i =0; i <  getProperty_actuator_list().size(); i++){
				if(model.getActuators().contains(get_actuator_list(i)))
					actuatorsByName.adoptAndAppend(&model.updActuators().get(get_actuator_list(i)));
				else
					cerr << "WARN: Controller::setup : Actuator " << get_actuator_list(i) << " was not found and will be ignored." << endl;
			}
			actuatorsByName.setMemoryOwner(false);
			setActuators(actuatorsByName);
		}
	}
}
