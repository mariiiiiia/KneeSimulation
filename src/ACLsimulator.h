#include <OpenSim/OpenSim.h>

#include <string>
#include <vector>

using namespace std;
using namespace SimTK;
using namespace OpenSim;


struct KneeLigamentsActivations
{
    union
    {
        double act[1];
        struct
        {
            double act_ACL;
        };
    };
};

struct ACLController
{
    /**
     * Called before every simulation step.
     * You get the current state of the simulation.
     * You can edit the activation.
     * @param time        [IN]  Elapsed time of simulation.
     * @param angles      [IN]  Vector with eyeball angles. (Euler X-Y-Z)
     * @param speeds      [IN]  Vector with eyeball angles' rate of change.
     * @param activations [OUT] Updatable vector of the 6 activations.
     */
    virtual void control (const double time,
        const double rotX,  const double rotY,  const double rotZ,
        const double rotXu, const double rotYu, const double rotZu,
        KneeLigamentsActivations &activations) = 0;
};


