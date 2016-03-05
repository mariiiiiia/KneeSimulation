#include "ACLsimulatorimpl.h"
#include "vegafem.h"
#include "addMeniscusToModel.h"
#include "addKneeContactGeometries.h"

#include <string>
#include <vector>

using namespace OpenSim;
using namespace SimTK;

int main(int argc, char *argv[])
{
	try {
		// Create an OpenSim model and set its name
		OpenSim::Model model("../resources/blender outputs in m/myKnee_changeBones.osim");

		SimTK::State &state = model.initSystem();

		// add meniscus bodies to left and right knee
		addMeniscusToKnee(model, true);
		addMeniscusToKnee(model, false);

		//Save the model to a file
		model.print("../resources/blender outputs in m/myKnee_addMenisci.osim");

		// add contact geometries at the right knee
		addKneeContactGeometries(model, false);

		//Save the model to a file
		model.print("../resources/blender outputs in m/myKnee_addContacts.osim");

		model.setUseVisualizer(1);
		state = model.initSystem();
		model.getVisualizer().show(state);

		//initSimulation();
		//forwardSim(model, state);
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

	return 1;
}
