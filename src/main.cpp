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

		//// add meniscus bodies to left and right knee
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

		// simulate
		//double kneeAngle [5] = {0, -15, -30, -60, -90};
		//for (int i=3; i<4; i++){
			//anteriorTibialLoadsFD(model, kneeAngle[i]);
		//}
		//performMCFD_atl(model, 40);
		//flexionFDSimulation(model);
		flexionFDSimulationWithHitMap(model);
		//performMCFD_flexion(model, 100);

		//OpenSim::Body* tibia = &model.updBodySet().get("tibia_r");
		//OpenSim::Body* block1 = new OpenSim::Body("block1", 1, SimTK::Vec3(0), 1*SimTK::Inertia(0.0));
		//OpenSim::Body* block2 = new OpenSim::Body("block2", 1, SimTK::Vec3(0), 1*SimTK::Inertia(0.0));
		//OpenSim::Body* block3 = new OpenSim::Body("block3", 1, SimTK::Vec3(0), 1*SimTK::Inertia(0.0));
		//OpenSim::Body* block4 = new OpenSim::Body("block4", 1, SimTK::Vec3(0), 1*SimTK::Inertia(0.0));
		//
		//DisplayGeometry * blockDG = new DisplayGeometry("blockMesh.obj");
		//blockDG->setColor(Vec3(1.0, 0.0, 0.0));
		//blockDG->setScaleFactors(SimTK::Vec3(0.05));
		//block1->updDisplayer()->updGeometrySet().cloneAndAppend(*blockDG);
		//blockDG->setColor(Vec3(0.0, 1, 0));
		//block2->updDisplayer()->updGeometrySet().cloneAndAppend(*blockDG);
		//blockDG->setColor(Vec3(0.2, 0.2, 1));
		//block3->updDisplayer()->updGeometrySet().cloneAndAppend(*blockDG);
		//blockDG->setColor(Vec3(1, 0, 1));
		//block4->updDisplayer()->updGeometrySet().cloneAndAppend(*blockDG);
		
		// tibial attachments
		//WeldJoint *weldJ1 = new WeldJoint("blockJ1", *tibia, SimTK::Vec3( 0.0227563, -0.0471042, 0.000484759), SimTK::Vec3(0), *block1, SimTK::Vec3(0), SimTK::Vec3(0), true); 
		//WeldJoint *weldJ2 = new WeldJoint("blockJ2", *tibia, SimTK::Vec3( 0.0172605, -0.0465166, 0.00250377), SimTK::Vec3(0), *block2, SimTK::Vec3(0), SimTK::Vec3(0), true); 
		//WeldJoint *weldJ3 = new WeldJoint("blockJ3", *tibia, SimTK::Vec3( 0.000827732, -0.0504358, 0.00752014), SimTK::Vec3(0), *block3, SimTK::Vec3(0), SimTK::Vec3(0), true); 
		//WeldJoint *weldJ4 = new WeldJoint("blockJ4", *tibia, SimTK::Vec3( -0.00295986, -0.049925, 0.00356567), SimTK::Vec3(0), *block4, SimTK::Vec3(0), SimTK::Vec3(0), true); 

		// femoral attachments
		//WeldJoint *weldJ1 = new WeldJoint("blockJ1", *tibia, SimTK::Vec3( 0.00427722, -0.404609, 0.0106371), SimTK::Vec3(0), *block1, SimTK::Vec3(0), SimTK::Vec3(0), true); 
		//WeldJoint *weldJ2 = new WeldJoint("blockJ2", *tibia, SimTK::Vec3( 0.00200603, -0.408992, 0.0122047), SimTK::Vec3(0), *block2, SimTK::Vec3(0), SimTK::Vec3(0), true); 
		//WeldJoint *weldJ3 = new WeldJoint("blockJ3", *tibia, SimTK::Vec3( 0.0128479, -0.41176, -0.001758403), SimTK::Vec3(0), *block3, SimTK::Vec3(0), SimTK::Vec3(0), true); 
		//WeldJoint *weldJ4 = new WeldJoint("blockJ4", *tibia, SimTK::Vec3( 0.00886686, -0.409824, -0.00681438), SimTK::Vec3(0), *block4, SimTK::Vec3(0), SimTK::Vec3(0), true); 
		
		// tibial load attachment
		//WeldJoint *weldJ4 = new WeldJoint("blockJ4", *tibia, SimTK::Vec3( 0.03, -0.03, 0.0), SimTK::Vec3(0), *block1, SimTK::Vec3(0), SimTK::Vec3(0), true); 

		//model.addBody(block1);
		//model.addBody(block2);
		//model.addBody(block3);
		//model.addBody(block4);

		//model.print("../resources/geometries/tibial_load_attachment.osim");

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
