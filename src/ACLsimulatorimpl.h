#include "ACLsimulator.h"


class ACLController_ : public OpenSim::Controller
{
    OpenSim_DECLARE_CONCRETE_OBJECT(ACLController_, Controller)

    ACLController *controller;

    vector<double> timesLog;
    vector<KneeLigamentsActivations> activationLog;

public:
    ACLController_ (ACLController *con) : controller(con)
    {
        setNumControls(6);
    }

    void reset () {timesLog.clear(); activationLog.clear();}

    const vector<double> &getControlTimesLog() {return timesLog;}

    const vector<KneeLigamentsActivations> &getControlLog() {return activationLog;}

    void computeControls(const State &s, Vector &controls) const OVERRIDE_11
    {
        KneeLigamentsActivations activations;

        controller->control(s.getTime(),
            s.getQ()[0], s.getQ()[1], s.getQ()[2],
            s.getU()[0], s.getU()[1], s.getU()[2],
            activations);

        for (int i=0; i<getNumControls(); ++i)
            controls[i] = activations.act[i];

        // Log activations (Need to drop const-ness)
        ACLController_ *this_ = const_cast<ACLController_*>(this);
        this_->timesLog.push_back(s.getTime());
        this_->activationLog.push_back(activations);
    }

};