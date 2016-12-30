#ifndef OSIMUTILS_H
#define OSIMUTILS_H

#include <string>
#include <OpenSim/OpenSim.h>
#include "CustomLigament.h"

using namespace std;
using namespace SimTK;
using namespace OpenSim;

#define echo(x) cout<<#x<<" = "<<x<<endl

class OsimUtils
{
public:
    Real static evalFunc(OpenSim::Function *f, Real x);

	void static enableAllForces(State &state, Model &model);

	void static disableAllForces(State &state, Model &model);

    void static writeFunctionsToFile(const vector<OpenSim::Function*>fs,
        const string filename,
        double dur, double step);

    void static writeFunctionsToFile(const vector<double> &times, 
        const vector<Vector> &acts, const string filename);

	void static writeFunctionsToFile(const vector<double> &times, 
		const vector<vector<double>> &acts, Storage *as, const string filename);

    void static writeForcesToFile(Model &model,
        const string filename,
        const Array<Vector> &forces, const Vector &times);
};


void printLigamentLengths(Model model, double knee_angle);

#endif