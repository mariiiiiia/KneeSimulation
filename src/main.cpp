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
		OpenSim::Model model("lower-limb(muscles millard13).osim");

		SimTK::State &state = model.initSystem();

		////Save the model to a file
		//model.print("myKnee_model.osim");

		forwardSim(model, state);
		std::cout << "OpenSim example completed successfully.\n";
		std::cin.get();
		
		return 0;
	}
	catch (OpenSim::Exception ex)
    {
        std::cout << "OpenSim exception\n" << ex.getMessage() << std::endl;
    }
    catch (std::exception ex)
    {
		std::cout << "std exception: " << ex.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
    }




    //QApplication a(argc, argv);
    //MainWindow w;
    //w.show();

    //return a.exec();
	return 1;
}
