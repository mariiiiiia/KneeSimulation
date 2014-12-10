#include "ACLsimulatorimpl.h"
#include "osimutils.h"

void forwardsim(Model model, SimTK::State& si){
	Vector activs_initial, activs_final;	
	vector<OpenSim::Function*> controlFuncs;
	double initialTime = 0.0;
	double finalTime = 2.0;

	si = model.initSystem();
    //model.equilibrateMuscles(si);
	
	double duration = 0.5;

	// compute activations of specific knee angle
	double knee_angle = -0.8; 
	computeActivations(model, knee_angle, controlFuncs, duration, false, activs_initial, activs_final, si);

	// add controller to the model after adding the control function
	KneeController *knee_controller = new KneeController( activs_final.size());
	//cout << "final activation size: " << activs_final.size() << "initial activ size: " << activs_initial.size() << endl;
	//cout << "control functions: " << controlFuncs.size() << endl;
	knee_controller->_activations.resize( activs_final.size());
	for (int i=0; i<activs_final.size(); i++){
		knee_controller->_activations.set( i, activs_final[i]);
	}
	knee_controller->setActuators(model.getActuators());
	model.addController(knee_controller);
	 
	// set model to initial state
	si = model.initSystem();

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
}


void computeActivations (Model &model, const double angle, vector<OpenSim::Function*> &controlFuncs, 
			double &duration, bool store, Vector &ssai, Vector &ssaf, State &si)
{
	calcSSact(model, ssai, si);
	
	//set movement parameters
	const CustomJoint &knee_r_joint = static_cast<const CustomJoint&>(model.getJointSet().get("knee_r"));
    knee_r_joint.get_CoordinateSet().get("knee_angle_r").setValue(si, angle);
	//model.equilibrateMuscles(si);

	calcSSact(model, ssaf, si);

	////--------------- DELETE THIS --------------------------------------
	//for (int i=0; i<ssaf.size(); i++){
	//	ssaf.set( i , 0.5);
	//}

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

    for (int i = 0; i < ssaf.size(); i++)  {
        double dssa = ssaf[i] - ssai[i];
        int j=0;

        values[j++] = ssai[i];
        values[j++] = values[j-1];

        values[j++] = ssaf[i] + dssa * 0.75;
        values[j++] = values[j-1];

        values[j++] = ssai[i] + dssa * 0.975;
        values[j++] = values[j-1];

        values[j++] = ssai[i] + dssa * 1.350;

        values[j++] = ssaf[i];
        values[j++] = values[j-1];

        OpenSim::Function *controlFunc = new PiecewiseLinearFunction(N, phases, values);
        controlFunc->setName(model.getActuators()[i].getName());
        controlFunc->setName("Excitation_" + model.getActuators()[i].getName());
        controlFuncs.push_back(controlFunc);
    }

    duration = t_eq + t_fix * 2;
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
		cout << as->getColumnLabels()[i] << ": " << activations[i] << endl;
		//cout << as->getDataColumn() << "\n" << endl;
	}
}