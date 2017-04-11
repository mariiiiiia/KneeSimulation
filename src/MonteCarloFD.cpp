#include <iostream>
#include <random>
#include <OpenSim/OpenSim.h>
#include "ACLsimulatorimpl.h"
#include <math.h>
//#include <OpenSim/Tools/InverseDynamicsTool.h>
//#include "cxxopts.hpp"

using namespace OpenSim;
using namespace SimTK;
using namespace std;
//using namespace cxxopts;

int process;

template<typename T> std::string changeToString(
    const T& value, int precision = std::numeric_limits<int>::infinity())
{
    std::ostringstream oss;
    if (precision != std::numeric_limits<int>::infinity())
    {
        oss << std::setprecision(precision);
    }     oss << value;     return oss.str();
}

void performMCFD(Model model, int iterations)
{
    // gaussian distribution setup
	//OpenSim::ElasticFoundationForce& meniscus_lat = static_cast<OpenSim::ElasticFoundationForce&>( model.updForceSet().get("femur_lat_meniscii_r"));
	//OpenSim::ElasticFoundationForce& meniscus_med = static_cast<OpenSim::ElasticFoundationForce&>( model.updForceSet().get("femur_med_meniscii_r"));
    //double stiffness = meniscus_lat.getStiffness();
	double stiffness;
    std::default_random_engine gen;
	std::normal_distribution<double> random_stiff(500, 500);

	// flexion controller 
	addFlexionController(model);

	// init system
	std::time_t result = std::time(nullptr);
	std::cout << "\nBefore initSystem() " << std::asctime(std::localtime(&result)) << endl;
	SimTK::State& si = model.initSystem();
	result = std::time(nullptr);
	std::cout << "\nAfter initSystem() " << std::asctime(std::localtime(&result)) << endl;
	
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
	model.equilibrateMuscles( si);

    for (int i = 0; i < iterations; i++)
    {
		//Model model_temp = Model(model);
		SimTK::State& si_temp = State(si);
		setKneeAngle(model, si_temp, 0);
		
		// Add reporters
		ForceReporter* forceReporter = new ForceReporter(&model);
		model.addAnalysis(forceReporter);
		CustomAnalysis* customReporter = new CustomAnalysis(&model, "r");
		model.addAnalysis(customReporter);

        //string outputFile = changeToString(i) + "_fd_.sto";
		double this_random_stiff = random_stiff(gen);
		cout << "before: " << this_random_stiff;
		this_random_stiff = std::ceil(this_random_stiff - 0.5);
		while (this_random_stiff <= 0)
		{
			this_random_stiff = random_stiff(gen);
			this_random_stiff = std::ceil(this_random_stiff - 0.5);
		}
		this_random_stiff *= 1.e8;
		static_cast<OpenSim::ElasticFoundationForce&>( model.updForceSet().get("femur_lat_meniscii_r")).setStiffness(this_random_stiff);
		static_cast<OpenSim::ElasticFoundationForce&>( model.updForceSet().get("femur_med_meniscii_r")).setStiffness(this_random_stiff);
		model.print("../outputs/MonteCarlo/NewModels/" + changeToString(i) + "newModel.osim");

		cout << " after: " << this_random_stiff << endl;

		// Create the integrator and manager for the simulation.
		SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
		//integrator.setAccuracy(1.0e-3);
		//integrator.setFixedStepSize(0.001);
		Manager manager(model, integrator);

		// Define the initial and final simulation times
		double initialTime = 0.0;
		double finalTime = 0.2;

		// Integrate from initial time to final time
		manager.setInitialTime(initialTime);
		manager.setFinalTime(finalTime);
		std::cout<<"\n\nIntegrating from "<<initialTime<<" to " <<finalTime<<". "<< i <<std::endl;

		result = std::time(nullptr);
		//std::cout << "\nBefore integrate(si) " << std::asctime(std::localtime(&result)) << endl;

		manager.integrate(si_temp);

		result = std::time(nullptr);
		//std::cout << "\nAfter integrate(si) " << std::asctime(std::localtime(&result)) << endl;

		// Save the simulation results
		Storage statesDegrees(manager.getStateStorage());
		//statesDegrees.print("../outputs/states_flexion.sto");
		model.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.setWriteSIMMHeader(true);
		statesDegrees.print("../outputs/MonteCarlo/states/" + changeToString(i) + "states_degrees_flexion.mot");
		// force reporter results
		forceReporter->getForceStorage().print("../outputs/MonteCarlo/ForceReporter/" + changeToString(i) + "force_reporter_flexion.mot");
		customReporter->print( "../outputs/MonteCarlo/CustomReporter/" + changeToString(i) + "custom_reporter_flexion.mot");
    }
}