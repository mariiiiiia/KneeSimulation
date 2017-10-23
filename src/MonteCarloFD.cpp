#include <iostream>
#include <random>
#include <OpenSim/OpenSim.h>
#include "ACLsimulatorimpl.h"
#include <math.h>
#include "osimutils.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

template<typename T> std::string changeToString(
    const T& value, int precision = std::numeric_limits<int>::infinity())
{
    std::ostringstream oss;
    if (precision != std::numeric_limits<int>::infinity())
    {
        oss << std::setprecision(precision);
    }     oss << value;     return oss.str();
}

void performMCFD_flexion(Model model, int iterations)
{
    std::default_random_engine gen;
	random_device rd;
    gen.seed(rd());
	//std::uniform_real_distribution<double> random_stiff(5*1.e8, 5*1.e9);
	//std::uniform_real_distribution<double> random_diss(0.0, 20.0);
	//std::uniform_real_distribution<double> random_aPCL_force(4800, 6200);
	std::uniform_real_distribution<double> random_PCL_length_range(0, 0.01); // length diff

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

	Array<double> samplingArray1;
	Array<double> samplingArray2;	

    for (int i = 50; i < iterations; i++)
    {
		// Add reporters
		ForceReporter* forceReporter = new ForceReporter(&model);
		model.addAnalysis(forceReporter);
		CustomAnalysis* customReporter = new CustomAnalysis(&model, "r");
		model.addAnalysis(customReporter);

        //string outputFile = changeToString(i) + "_fd_.sto";
		//double this_random_stiff = random_stiff(gen);
		//double this_random_diss = random_diss(gen);
		//double this_random_aPCL = random_aPCL_force(gen);
		//double this_random_pPCL = this_random_aPCL * 0.8363;
		double this_random_PCL_diff = random_PCL_length_range(gen);
		double this_random_aPCL_length = (0.031 - 0.005) + this_random_PCL_diff;
		double this_random_pPCL_length = (0.030 - 0.005) + this_random_PCL_diff;

		//static_cast<OpenSim::ElasticFoundationForce&>( model.updForceSet().get("femur_lat_meniscii_r")).setStiffness(this_random_stiff);
		//static_cast<OpenSim::ElasticFoundationForce&>( model.updForceSet().get("femur_med_meniscii_r")).setStiffness(this_random_stiff);
		//static_cast<OpenSim::ElasticFoundationForce&>( model.updForceSet().get("femur_lat_meniscii_r")).setDissipation(this_random_diss);
		//static_cast<OpenSim::ElasticFoundationForce&>( model.updForceSet().get("femur_med_meniscii_r")).setDissipation(this_random_diss);
		//static_cast<OpenSim::CustomLigament&>( model.updForceSet().get("aPCL_R")).set_stiffness(this_random_aPCL);
		//static_cast<OpenSim::CustomLigament&>( model.updForceSet().get("pPCL_R")).set_stiffness(this_random_pPCL);
		static_cast<OpenSim::CustomLigament&>( model.updForceSet().get("aPCL_R")).setRestingLength(this_random_aPCL_length);
		static_cast<OpenSim::CustomLigament&>( model.updForceSet().get("pPCL_R")).setRestingLength(this_random_pPCL_length);

		samplingArray1.append(this_random_aPCL_length);
		samplingArray2.append(this_random_pPCL_length);

		setHipAngle(model, si, 90);
		setKneeAngle(model, si, 0, false, false);
		si = model.initSystem();
		
		
		// Create the integrator and manager for the simulation.
		SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
		//integrator.setAccuracy(1.0e-3);
		//integrator.setFixedStepSize(0.0001);
		Manager manager(model, integrator);

		// Define the initial and final simulation times
		double initialTime = 0.0;
		double finalTime = 0.25;

		// Integrate from initial time to final time
		manager.setInitialTime(initialTime);
		manager.setFinalTime(finalTime);
		std::cout<<"\n\nIntegrating from "<<initialTime<<" to " <<finalTime<<". "<< i <<std::endl;

		result = std::time(nullptr);
		std::cout << "\nBefore integrate(si) " << std::asctime(std::localtime(&result)) << endl;

		manager.integrate(si);

		result = std::time(nullptr);
		std::cout << "\nAfter integrate(si) " << std::asctime(std::localtime(&result)) << endl;

		// Save the simulation results
		Storage statesDegrees(manager.getStateStorage());
		//statesDegrees.print("../outputs/MonteCarlo/states_rads/" + changeToString(i) +  "_states_flexion.sto");
		model.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.setWriteSIMMHeader(true);
		statesDegrees.print("../outputs/MC_flexion/MC_PclLength_Flexion_v6/states/" + changeToString(i) + "_states_degrees_flexion.mot");
		// force reporter results
		forceReporter->getForceStorage().print("../outputs/MC_flexion/MC_PclLength_Flexion_v6/ForceReporter/" + changeToString(i) + "_force_reporter_flexion.mot");
		customReporter->print( "../outputs/MC_flexion/MC_PclLength_Flexion_v6/CustomReporter/" + changeToString(i) + "_custom_reporter_flexion.mot");

		model.removeAnalysis(forceReporter);
		model.removeAnalysis(customReporter);

		writeArrayToFile("../outputs/MC_flexion/MC_PclLength_Flexion_v6/aPCL_length.txt", samplingArray1);
		writeArrayToFile("../outputs/MC_flexion/MC_PclLength_Flexion_v6/pPCL_length.txt", samplingArray2);
    }

}

void performMCFD_atl(Model model, int iterations)
{
	// gaussian distribution setup
    std::default_random_engine gen;
	random_device rd;
    gen.seed(rd());
	//std::uniform_real_distribution<double> random_stiff(5*1.e8, 5*1.e9);
	//std::uniform_real_distribution<double> random_diss(0.0, 20.0);
	//std::uniform_real_distribution<double> random_aACL_force(1300,1700);
	std::uniform_real_distribution<double> random_ACL_length_range(0, 0.005); // length diff

	// init system
	std::time_t result = std::time(nullptr);
	std::cout << "\nBefore initSystem() " << std::asctime(std::localtime(&result)) << endl;
	SimTK::State& si = model.initSystem();
	result = std::time(nullptr);
	std::cout << "\nAfter initSystem() " << std::asctime(std::localtime(&result)) << endl;
	
	// set gravity	
	model.updGravityForce().setGravityVector(si, Vec3(0,0,0));

	model.equilibrateMuscles( si);

	Array<double> samplingArray1;
	Array<double> samplingArray2;	

    for (int i = 20; i < iterations; i++)
    {
		double kneeAngle [6] = {0, -15, -30, -60, -90};

		//double this_random_stiff = random_stiff(gen);
		//double this_random_diss = random_diss(gen);
		//double this_random_aACL = random_aACL_force(gen);
		//double this_random_pACL = this_random_aACL * 1.266;
		double this_random_ACL_diff = random_ACL_length_range(gen);
		double this_random_aACL_length = (0.031 - 0.0025) + this_random_ACL_diff;
		double this_random_pACL_length = (0.0253 - 0.0025) + this_random_ACL_diff;

		//static_cast<OpenSim::ElasticFoundationForce&>( model.updForceSet().get("femur_lat_meniscii_r")).setStiffness(this_random_stiff);
		//static_cast<OpenSim::ElasticFoundationForce&>( model.updForceSet().get("femur_med_meniscii_r")).setStiffness(this_random_stiff);
		//static_cast<OpenSim::ElasticFoundationForce&>( model.updForceSet().get("femur_lat_meniscii_r")).setDissipation(this_random_diss);
		//static_cast<OpenSim::ElasticFoundationForce&>( model.updForceSet().get("femur_med_meniscii_r")).setDissipation(this_random_diss);
		//static_cast<OpenSim::CustomLigament&>( model.updForceSet().get("aACL_R")).set_stiffness(this_random_aACL);
		//static_cast<OpenSim::CustomLigament&>( model.updForceSet().get("pACL_R")).set_stiffness(this_random_pACL);
		static_cast<OpenSim::CustomLigament&>( model.updForceSet().get("aACL_R")).setRestingLength(this_random_aACL_length);
		static_cast<OpenSim::CustomLigament&>( model.updForceSet().get("pACL_R")).setRestingLength(this_random_pACL_length);

		samplingArray1.append(this_random_aACL_length);
		samplingArray2.append(this_random_pACL_length);

		for (int j=3; j<4; j++)
		{
			// Add reporters
			ForceReporter* forceReporter = new ForceReporter(&model);
			model.addAnalysis(forceReporter);
			CustomAnalysis* customReporter = new CustomAnalysis(&model, "r");
			model.addAnalysis(customReporter);

			//string outputFile = changeToString(i) + "_fd_.sto";

			// edit prescribed force for anterior tibial load
			addTibialLoads(model, kneeAngle[j]);	
			si = model.initSystem();
			setKneeAngle(model, si, kneeAngle[j], true, true);
		
			// Create the integrator and manager for the simulation.
			SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
			//integrator.setAccuracy(1.0e-3);
			//integrator.setFixedStepSize(0.0001);
			Manager manager(model, integrator);

			// Define the initial and final simulation times
			double initialTime = 0.0;
			double finalTime = 0.8;

			// Integrate from initial time to final time
			manager.setInitialTime(initialTime);
			manager.setFinalTime(finalTime);
			std::cout<<"\n\nIntegrating from "<<initialTime<<" to " <<finalTime<<". "<< i <<std::endl;

			result = std::time(nullptr);
			std::cout << "\nBefore integrate(si) " << std::asctime(std::localtime(&result)) << endl;

			manager.integrate(si);

			result = std::time(nullptr);
			std::cout << "\nAfter integrate(si) " << std::asctime(std::localtime(&result)) << endl;

			// Save the simulation results
			Storage statesDegrees(manager.getStateStorage());
			//statesDegrees.print("../outputs/MonteCarlo/states_rads/" + changeToString(i) +  "_states_flexion.sto");
			model.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
			statesDegrees.setWriteSIMMHeader(true);
			statesDegrees.print("../outputs/MC_anterior_loads/MC_AclLength_Atl_v6/states/" + changeToString(i) + "_states_degrees_atl_" + changeToString(abs(kneeAngle[j])) + ".mot" );
			// force reporter results
			forceReporter->getForceStorage().print("../outputs/MC_anterior_loads/MC_AclLength_Atl_v6/ForceReporter/" + changeToString(i) + "_force_reporter_atl_" + changeToString(abs(kneeAngle[j])) + ".mot");
			customReporter->print( "../outputs/MC_anterior_loads/MC_AclLength_Atl_v6/CustomReporter/" + changeToString(i) + "_custom_reporter_atl_" + changeToString(abs(kneeAngle[j])) + ".mot");

			model.removeAnalysis(forceReporter);
			model.removeAnalysis(customReporter);

			writeArrayToFile("../outputs/MC_anterior_loads/MC_AclLength_Atl_v6/aACL_length.txt", samplingArray1);
			writeArrayToFile("../outputs/MC_anterior_loads/MC_AclLength_Atl_v6/pACL_length.txt", samplingArray2);
		}
    }

}