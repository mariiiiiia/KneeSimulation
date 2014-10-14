#include "mainwindow.h"
//#include "ACLsimulator.h"
#include "ACLsimulatorimpl.h"
#include <QApplication>

#include <OpenSim/OpenSim.h>


using namespace OpenSim;
using namespace SimTK;

int main(int argc, char *argv[])
{
	try {
		// Create an OpenSim model and set its name
		Model model("lower-limb(muscles millard13).osim");

		// Save the model to a file
		model.print("myKnee_model.osim");

	    // Create OpenSim controller object.
		ACLController *aclController;
		ACLController_ *controller = new ACLController_(aclController);
		controller->setActuators(model.getActuators());
		model.addController(controller);

		// Initialize the system
		SimTK::State& state = model.initSystem();

		// Set initial parameters.
		const CustomJoint &pelvisj = static_cast<const CustomJoint&>(model.getJointSet().get("ground-pelvis"));
		const CoordinateSet &pelvisjcs = pelvisj.get_CoordinateSet();
		pelvisjcs.get("pelvis_ty").setValue( state, 0.5);

		model.equilibrateMuscles(state);

	    // Add reporters
		ForceReporter* forceReporter = new ForceReporter(&model);
		model.addAnalysis(forceReporter);

	    // Simulate
		RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
		Manager manager(model, integrator);
		manager.setInitialTime(0);
		manager.setFinalTime(4.0);
		manager.integrate(state);


		// Save the simulation results
		/*Storage statesDegrees(manager.getStateStorage());
		statesDegrees.print("myknee_model.sto");
		model.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.setWriteSIMMHeader(true);
		statesDegrees.print("myknee_model_states_degrees.mot");*/


		return 0;
	}
	catch (OpenSim::Exception ex)
    {
        std::cout << ex.getMessage() << std::endl;
    }
    catch (std::exception ex)
    {
        std::cout << ex.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
    }

    std::cout << "OpenSim example completed successfully.\n";
	std::cin.get();


    //QApplication a(argc, argv);
    //MainWindow w;
    //w.show();

    //return a.exec();
	return 1;
}
