#include "ACLsimulatorimpl.h"
#include "addBodies.h"
#include "addKneeContacts.h"
#include "CustomLigament.h"
#include <ctime>
//#include "cxxopts.hpp"
#include "MonteCarloFD.h"
#include <math.h>
#include <random>

#include <string>
#include <vector>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

#define PAUSE 0

/*
*	ToDo:
*		validate anterior forces
*		validate mass / inertia of tibia upper, meniscus, femur
*/

int main(int argc, char *argv[])
{
	try 
	{
		Object::registerType(CustomLigament());
		// Create an OpenSim model and set its name
		OpenSim::Model model("../resources/geometries/closed_knee_ligaments_0_3_scheburn_ant_loads.osim");

		//// add meniscus bodies to left and right knee
		//cout << "Adding meniscus" << endl;
		//addMeniscusWeldJoints(model, true);
		//addMeniscusWeldJoints(model, false);
		//// add femur bodies and weld joints 
		//cout << "Adding femur" << endl;
		//addFemurWeldJoints(model, true);
		//addFemurWeldJoints(model, false);
		//// add tibia weld joint
		//cout << "Addint tibia upper bodies" << endl;
		////addTibiaWeldJoints(model, true);
		////addTibiaWeldJoints(model, false);
		//addUpperTibiaFreeJoints( model, true);
		//addUpperTibiaFreeJoints( model, false);
		//// add contact geometries at the right knee
		//cout << "Adding contacts" << endl;
		//addKneeContactGeometries(model, true);
		//addKneeContactGeometries(model, false);	
		//// add contact forces
		//cout << "Adding contact forces" << endl;
		//addEFForce(model, 1.E12, 1.0, 0.8, 0.04, 0.04, true);
		//addEFForce(model, 1.E12, 1.0, 0.8, 0.04, 0.04, false);	
		//model.print("../resources/geometries/closed_knee_ligaments_1_0.osim");
		// add patella custom joint
		
		//printLigamentLengths(model);

		// simulate
		//inverseSimulation(model);
		//staticOptimization(model);
		//anteriorTibialLoadsFD(model);
		forwardSimulation(model);

		//performMCFD(model, 100);

		 //Save the model to a file
		//model.setUseVisualizer(1);
		//SimTK::State& state = model.initSystem();
		//state = model.initSystem();
		//model.getVisualizer().show(state);

		std::cout << "OpenSim example completed successfully.\n";
		std::cin.get();

		return 0;
	}
	catch (OpenSim::Exception ex)
    {
        std::cout << "OpenSim exception\n" << ex.getMessage() << std::endl;
    }
	catch (SimTK::Exception::ErrorCheck ex)
	{
		cout << "Simbody exception\n" << ex.getMessage() << endl;
	}
    catch (std::exception ex)
    {
		std::cout << "std exception: " << ex.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
    }

	std::cin.get();
	return 1;
}
