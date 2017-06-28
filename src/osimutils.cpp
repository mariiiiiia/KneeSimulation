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

void OsimUtils::writeLengthsToFile(string filename, const Array<Array<double>> lengths, const Array<double> angle)
{
	 // Write result forces to file
    ofstream file(filename.c_str());

    // Labels
    file << "angles\t";
	for (int i=0; i<angle.size(); i++)
	{
		file << "'" << angle[i] << "'\t";
	}
	file << endl;

	//ligament names
	Array<string> names;
	names.append("aACL");
	names.append("pACL");
	names.append("aPCL");
	names.append("pPCL");
	names.append("aMCL");
	names.append("mMCL");
	names.append("pMCL");
	names.append("aLCL");
	names.append("mLCL");
	names.append("pLCL");

    // Data
    for (int l = 0; l < lengths.size(); l++) {
		file << names.get(l);
        for (int f = 0; f < angle.size(); f++)
		{
			//cout << lengths[f][l] << endl;
            file << "\t" << lengths[l][f];
		}
		file << endl;
    }

    file.close();
}

void printLigamentLengths(Model model)
{
	//Object::registerType(CustomLigament());
	Array<double> aACL_R_lengths, pACL_R_lengths, aPCL_R_lengths, pPCL_R_lengths, aMCL_R_lengths, mMCL_R_lengths, pMCL_R_lengths,\
		aLCL_R_lengths, mLCL_R_lengths, pLCL_R_lengths;
	Array<Array<double>> lengths;

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

	aACL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aACL_R")).getLength(si));
	pACL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pACL_R")).getLength(si));
	aPCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aPCL_R")).getLength(si));
	pPCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pPCL_R")).getLength(si));
	aMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aMCL_R")).getLength(si));
	mMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("mMCL_R")).getLength(si));
	pMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pMCL_R")).getLength(si));
	aLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aLCL_R")).getLength(si));
	mLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("mLCL_R")).getLength(si));
	pLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pLCL_R")).getLength(si));

	cout << "knee angle: " << -30 << endl;
	knee_r_cs.get("knee_angle_r").setValue(si, -0.52359877); 
	//knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.01); 

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

	aACL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aACL_R")).getLength(si));
	pACL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pACL_R")).getLength(si));
	aPCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aPCL_R")).getLength(si));
	pPCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pPCL_R")).getLength(si));
	aMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aMCL_R")).getLength(si));
	mMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("mMCL_R")).getLength(si));
	pMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pMCL_R")).getLength(si));
	aLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aLCL_R")).getLength(si));
	mLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("mLCL_R")).getLength(si));
	pLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pLCL_R")).getLength(si));

	cout << "knee angle: " << -60 << endl;
	knee_r_cs.get("knee_angle_r").setValue(si, -1.0471975);  
	//knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.0207306); 

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

	aACL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aACL_R")).getLength(si));
	pACL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pACL_R")).getLength(si));
	aPCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aPCL_R")).getLength(si));
	pPCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pPCL_R")).getLength(si));
	aMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aMCL_R")).getLength(si));
	mMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("mMCL_R")).getLength(si));
	pMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pMCL_R")).getLength(si));
	aLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aLCL_R")).getLength(si));
	mLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("mLCL_R")).getLength(si));
	pLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pLCL_R")).getLength(si));

	cout << "knee angle: " << -90 << endl;
	knee_r_cs.get("knee_angle_r").setValue(si, -1.57079632);  
	//knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.028); 

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

	aACL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aACL_R")).getLength(si));
	pACL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pACL_R")).getLength(si));
	aPCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aPCL_R")).getLength(si));
	pPCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pPCL_R")).getLength(si));
	aMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aMCL_R")).getLength(si));
	mMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("mMCL_R")).getLength(si));
	pMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pMCL_R")).getLength(si));
	aLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aLCL_R")).getLength(si));
	mLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("mLCL_R")).getLength(si));
	pLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pLCL_R")).getLength(si));

	cout << "knee angle: " << -120 << endl;
	knee_r_cs.get("knee_angle_r").setValue(si, -2.09439510);  
	//knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.030837); 

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

	aACL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aACL_R")).getLength(si));
	pACL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pACL_R")).getLength(si));
	aPCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aPCL_R")).getLength(si));
	pPCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pPCL_R")).getLength(si));
	aMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aMCL_R")).getLength(si));
	mMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("mMCL_R")).getLength(si));
	pMCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pMCL_R")).getLength(si));
	aLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("aLCL_R")).getLength(si));
	mLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("mLCL_R")).getLength(si));
	pLCL_R_lengths.append( static_cast<const CustomLigament&>(model.getForceSet().get("pLCL_R")).getLength(si));

	lengths.append( aACL_R_lengths); lengths.append( pACL_R_lengths); lengths.append( aPCL_R_lengths); lengths.append( pPCL_R_lengths); 
	lengths.append( aMCL_R_lengths); lengths.append( mMCL_R_lengths); lengths.append( pMCL_R_lengths); 
	lengths.append( aLCL_R_lengths); lengths.append( mLCL_R_lengths); lengths.append( pLCL_R_lengths);

	Array<double> angle;
	angle.append( 0);
	angle.append( -30);
	angle.append( -60);
	angle.append( -90);
	angle.append( -120); 
	OsimUtils::writeLengthsToFile("../outputs/flexion_lengths.txt", lengths, angle);
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