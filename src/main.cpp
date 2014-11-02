#include <QApplication>
#include "ACLsimulatorimpl.h"

#include <string>
#include <vector>

using namespace OpenSim;
using namespace SimTK;

int main(int argc, char *argv[])
{
	try {
		// Create an OpenSim model and set its name
		OpenSim::Model model("lower-limb(muscles millard13).osim", false);

		SimTK::State &state = model.initSystem();

		// Save the model to a file
		//model.print("myKnee_model.osim");

		forwardsim(model, state);

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
