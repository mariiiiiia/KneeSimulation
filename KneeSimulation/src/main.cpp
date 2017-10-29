#include "ACLsimulatorimpl.h"
#include "addBodies.h"
#include "addKneeContacts.h"
#include "CustomLigament.h"
#include <ctime>
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
		OpenSim::Model model("../resources/3DGaitModel2392_optimized_v6.osim");

		/*
		*	CHANGE MODEL BODIES AND PROPERTIES
		*/
		// add meniscus bodies to left and right knee
		//cout << "Adding meniscus" << endl;
		//addMeniscusWeldJoints(model, false);
		//// add femur bodies and weld joints 
		//cout << "Adding femur" << endl;
		//addFemurWeldJoints(model, false);
		//// add tibia weld joint
		//cout << "Addint tibia upper bodies" << endl;
		////addTibiaWeldJoints(model, false);
		//// add contact geometries at the right knee
		//cout << "Adding contacts" << endl;
		//addKneeContactGeometries(model, false);	
		//// add contact forces
		//cout << "Adding contact forces" << endl;
		//addEFForces(model, 1.E12, 1.0, 0.8, 0.04, 0.04, 1.E11, 1.0, 0.5, 0.03, 0.03, false);	
		//model.print("../resources/geometries/closed_knee_ligaments_1_0.osim");	
		//printLigamentLengths(model);

		/*
		*	ANTERIOR TIBIAL LOADS SIMULATION
		*/
		//double kneeAngle [5] = {0, -15, -30, -60, -90};
		//for (int i=0; i<5; i++){
		//	anteriorTibialLoadsFD(model, kneeAngle[i]);
		//}

		/*
		*	ANTERIOR TIBIAL LOADS MONTE CARLO ANALYSIS
		*/
		//performMCFD_atl(model, 40);

		/*
		*	ACTIVE KNEE FLEXION SIMULATION
		*/
		//flexionFDSimulation(model);
		
		/*
		*	ACTIVE KNEE FLEXION MONTE CARLO ANALYSIS
		*/
		//performMCFD_flexion(model, 100);

		/*
		*	PERFORM A KNEE TASK AND VISUALIZE ARTICULAR CONTACT POINTS (ON TIBIA AND FEMUR)
		*/
		flexionFDSimulationWithHitMap(model);

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
