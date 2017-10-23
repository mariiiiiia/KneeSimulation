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

void printLigamentLengthsInExtension(Model model)
{
	//Object::registerType(CustomLigament());

	SimTK::State& si = model.initSystem();
	
	const CoordinateSet &knee_r_cs = model.getJointSet().get("knee_r").getCoordinateSet();
	knee_r_cs.get("knee_angle_r").setValue(si, -0.0290726);  
	
	cout << "knee angle: " << 0 << endl;
	cout << "aACL_R length: " << static_cast<const CustomLigament&>(model.getForceSet().get("aACL_R")).getLength(si) << endl;
	cout << "pACL_R length: " << static_cast<const CustomLigament&>(model.getForceSet().get("pACL_R")).getLength(si) << endl;
	cout << "aPCL_R length: " << static_cast<const CustomLigament&>(model.getForceSet().get("aPCL_R")).getLength(si) << endl;
	cout << "pPCL_R length: " << static_cast<const CustomLigament&>(model.getForceSet().get("pPCL_R")).getLength(si) << endl;
	cout << "aMCL_R length: " << static_cast<const CustomLigament&>(model.getForceSet().get("aMCL_R")).getLength(si) << endl;
	cout << "mMCL_R length: " << static_cast<const CustomLigament&>(model.getForceSet().get("mMCL_R")).getLength(si) << endl;
	cout << "pMCL_R length: " << static_cast<const CustomLigament&>(model.getForceSet().get("pMCL_R")).getLength(si) << endl;
	cout << "aLCL_R length: " << static_cast<const CustomLigament&>(model.getForceSet().get("aLCL_R")).getLength(si) << endl;
	cout << "mLCL_R length: " << static_cast<const CustomLigament&>(model.getForceSet().get("mLCL_R")).getLength(si) << endl;
	cout << "pLCL_R length: " << static_cast<const CustomLigament&>(model.getForceSet().get("pLCL_R")).getLength(si) << endl;
}

void writeArrayToFile(string filename, const Array<double> myArray)
{
	// Write result forces to file
    ofstream file(filename.c_str());

    // Labels
    file << "values: " << endl;
    for (int i = 0; i < myArray.getSize(); i++) {
        file << myArray[i] << endl;
    }

    file.close();

}