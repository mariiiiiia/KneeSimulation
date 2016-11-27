#include "osimutils.h"

template <class charT, charT sep> class punct_facet : public std::numpunct<charT> {
protected:
    charT do_decimal_point() const { return sep; }
};

// OUT: return f[x]
Real OsimUtils::evalFunc(OpenSim::Function *f, Real x)
{
    Vector xv(1); xv[0] = x;
    return f->calcValue(xv);
}

void OsimUtils::disableAllForces(State &state, Model &model)
{
    for (int i = 0; i < model.getForceSet().getSize(); i++)
        model.updForceSet().get(i).setDisabled(state, true);
}

void OsimUtils::enableAllForces(State &state, Model &model)
{
    for (int i = 0; i < model.getForceSet().getSize(); i++)
        model.updForceSet().get(i).setDisabled(state, false);
}

void OsimUtils::writeFunctionsToFile(const vector<OpenSim::Function*>fs,
    const string filename, double dur, double step)
{
    ofstream file(filename.c_str());

    // Set decimal separator character
    file.imbue(std::locale(file.getloc(), new punct_facet<char, '.'>));

    file << "nRows=" << (int)(dur/step+1) << "\n";
    file << "nColumns=" << (1+fs.size()) << "\n";
    file << "endheader" << "\n\n";
    file << "time" << " ";
    for (unsigned i=0; i< fs.size(); i++) file << fs[i]->getName() << ' ';
    file << "\n";

    Vector xv(1);
    for (double t = 0; t <= dur; t += step) {
        xv[0] = t;
        file << t << "\t";
        for (unsigned i=0; i< fs.size(); i++) file << fs[i]->calcValue(xv) << '\t';
        file << "\n";
    }

    file << endl;
}

void OsimUtils::writeFunctionsToFile(const vector<double> &times, 
    const vector<Vector> &acts, const string filename)
{
    ofstream file(filename.c_str());

    // Set decimal separator character
    file.imbue(std::locale(file.getloc(), new punct_facet<char, '.'>));

    file << "nRows=" << times.size() << "\n";
    file << "nColumns=" << acts[0].size() +1 << "\n";
    file << "endheader" << "\n\n";
    file << "time" << " ";
    for (int i=0; i< acts[0].size(); i++) file << "Act_" << i << ' ';
    file << "\n";

    Vector xv(1);
    for (double t = 0; t<times.size(); t++) {
        file << times[t] << "\t";
		for (int i=0; i<acts[0].size(); i++) 
            file << acts[t][i] << '\t';
        file << "\n";
    }

    file << endl;
}

void OsimUtils::writeFunctionsToFile(const vector<double> &times, 
    const std::vector<std::vector<double>> &acts, Storage *as, const string filename)
{
    ofstream file(filename.c_str());

    // Set decimal separator character
    file.imbue(std::locale(file.getloc(), new punct_facet<char, '.'>));

    file << "nRows=" << times.size() << "\n";
    file << "nColumns=" << acts.at(0).size() +1 << "\n";
    file << "endheader" << "\n\n";
    file << "time" << "\t";
	
    for (unsigned i=1; i< acts.at(0).size(); i++) 
		file << as->getColumnLabels().get(i).c_str() << "\t";
    file << "\n";

    Vector xv(1);
	cout << acts.at(0).at(0) << " " << acts.at(1).at(0) << endl;
    for (double t = 0; t<times.size(); t++) {
        //file << times[t] << "\t";
		for (unsigned i=0; i<acts.at(0).size(); i++) 
		{
            file << acts.at(t).at(i) << '\t';
		}
        file << "\n";
    }

    file << endl;
}

void OsimUtils::writeForcesToFile(Model &model,const string filename,
    const Array<Vector> &forces, const Vector &times)
{
    // Write result forces to file
    ofstream file(filename.c_str());

    // Labels
    file << "times ";
    for (int i = 0; i < model.getCoordinateSet().getSize(); i++) {
        bool r = model.getCoordinateSet()[i].getMotionType() == Coordinate::Rotational;
        file << model.getCoordinateSet().get(i).getName()+(r?"_moment":"_force")<<" ";
    }
    file << endl;

    // Data
    for (int l = 0; l < forces.size(); l++) {
        file << times[l] << "\t";
        for (int f = 0; f < forces[l].size(); f++)
            file << forces[l][f] << "\t";
        file << endl;
    }

    file.close();
}


void printLigamentLengths(Model& model)
{
	Object::registerType(CustomLigament());

	SimTK::State& si = model.initSystem();

	const CustomLigament &aACL_R = static_cast<const CustomLigament&>(model.getForceSet().get("aACL_R"));
	const CustomLigament &pACL_R = static_cast<const CustomLigament&>(model.getForceSet().get("pACL_R"));
	const CustomLigament &aPCL_R = static_cast<const CustomLigament&>(model.getForceSet().get("aPCL_R"));
	const CustomLigament &pPCL_R = static_cast<const CustomLigament&>(model.getForceSet().get("pPCL_R"));
	const CustomLigament &aMCL_R = static_cast<const CustomLigament&>(model.getForceSet().get("aMCL_R"));
	const CustomLigament &aLCL_R = static_cast<const CustomLigament&>(model.getForceSet().get("aLCL_R"));

	const CustomLigament &aPCL_L = static_cast<const CustomLigament&>(model.getForceSet().get("aPCL_L"));
	const CustomLigament &pPCL_L = static_cast<const CustomLigament&>(model.getForceSet().get("pPCL_L"));
	const CustomLigament &aMCL_L = static_cast<const CustomLigament&>(model.getForceSet().get("aMCL_L"));
	const CustomLigament &aLCL_L = static_cast<const CustomLigament&>(model.getForceSet().get("aLCL_L"));

	cout << "aACL_R length: " << aACL_R.getLength(si) << endl;
	cout << "pACL_R length: " << pACL_R.getLength(si) << endl;
	cout << "aPCL_R length: " << aPCL_R.getLength(si) << endl;
	cout << "pPCL_R length: " << pPCL_R.getLength(si) << endl;
	cout << "aMCL_R length: " << aMCL_R.getLength(si) << endl;
	cout << "aLCL_R length: " << aLCL_R.getLength(si) << endl;
	
	cout << "aPCL_L length: " << aPCL_L.getLength(si) << endl;
	cout << "pPCL_L length: " << pPCL_L.getLength(si) << endl;
	cout << "aMCL_L length: " << aMCL_L.getLength(si) << endl;
	cout << "aLCL_L length: " << aLCL_L.getLength(si) << endl;

	//CustomLigament aACL_r = static_cast<CustomLigament>( aACL_R);
	//CustomLigament pACL_r = static_cast<CustomLigament>( pACL_R);
	//CustomLigament aPCL_r = static_cast<CustomLigament>( aPCL_R);
	//CustomLigament pPCL_r = static_cast<CustomLigament>( pPCL_R);
	//CustomLigament aMCL_r = static_cast<CustomLigament>( aMCL_R);
	//CustomLigament aLCL_r = static_cast<CustomLigament>( aLCL_R);

	//if ( !aACL_r.setRestingLength( aACL_R.getLength(si)))
	//	cout << "error" << endl;
	//pACL_r.setRestingLength( pACL_R.getLength(si));
	//aPCL_r.setRestingLength( aPCL_R.getLength(si));
	//pPCL_r.setRestingLength( pPCL_R.getLength(si));
	//aMCL_r.setRestingLength( aMCL_R.getLength(si));
	//aLCL_r.setRestingLength( aLCL_R.getLength(si));
	//
	//aPCL_L.setRestingLength( aPCL_L.getLength(si));
	//pPCL_L.setRestingLength( pPCL_L.getLength(si));
	//aMCL_L.setRestingLength( aMCL_L.getLength(si));
	//aLCL_L.setRestingLength( aLCL_L.getLength(si));
}