#ifndef CUSTOMANALYSIS_H
#define CUSTOMANALYSIS_H

#include <string>

#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimPluginDLL.h"

#define SAVE_FORCE

namespace OpenSim { 

class OSIMPLUGIN_API CustomAnalysis : public Analysis 
{
OpenSim_DECLARE_CONCRETE_OBJECT(CustomAnalysis, Analysis);
public:
//=======================================================================
// PROPERTIES
//=======================================================================
/** @name Property declarations
	These are the serializable properties associated with this class. **/
/**@{**/

    /** String list property containing the name of the body names*/
    //OpenSim_DECLARE_LIST_PROPERTY(body_names, std::string, 
    //"Names of the bodies on which to perform the analysis."
    //"The key word 'All' indicates that the analysis should be performed for all bodies.");

    // Here are some examples of other scalar property types.
    // Uncomment them as you need them.
    // ------------------------------------------------------
    //// My string property
    //OpenSim_DECLARE_PROPERTY(string_property, std::string, 
    //"My string property."); 

    //// My int property
    //OpenSim_DECLARE_PROPERTY(int_property, int, 
    //"My int property."); 

    //// My bool property
    //OpenSim_DECLARE_PROPERTY(bool_property, bool, 
    //"My bool property."); 

    //// My double property
    //OpenSim_DECLARE_PROPERTY(double_property, double, 
    //"My double property."); 

/**@}**/


//=======================================================================
// INTERNAL MEMBER VARIABLES
//=======================================================================

	// In addition to properties, add any additional member variables
	// you need for your analysis.  These variables are not read from
	// or written to file.  They are just variables you use to execute
	// your analysis.  For example, you will almost certainly need a
	// storage object for storing the results of your analysis.

	// Storage object for storing and writing out results.  In general,
	// each storage file that you create will contain only one kind of data.
	// Create a different storage object for each kind of data.  For example,
	// create a _storePos for positions, _storeVel for velocities,
	// _storeAcc for accelerations, etc. */

	SimTK::Vec3 m_initial_position;

	/** Storage for recording body positions. */
	Storage m_storage;

	std::string m_leg;
//=============================================================================
// METHODS
//=============================================================================
private:
    /** Construct default values for internal member variables, */
	/** i.e., zero data and set pointers to Null */
	void setNull();

	/** Construct default values for properties */
	void constructProperties();

public:

	CustomAnalysis();

    /** Default constructor */
    CustomAnalysis(Model *model, std::string leg);

	void get_displacement_column(Array<double> &force, Array<double> &displacement);

    /** setModel */
	virtual void setModel(Model& aModel);

	//-------------------------------------------------------------------------
	// METHODS THAT MUST BE OVERRIDDEN
	//-------------------------------------------------------------------------
	virtual int begin(SimTK::State& s);
	virtual int step(const SimTK::State& s, int stepNumber);
	virtual int	end(SimTK::State& s);

	//-------------------------------------------------------------------------
	// IO
	//-------------------------------------------------------------------------
	virtual int print(const std::string &path);
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");


protected:
	//========================== Internal Methods =============================
	int record(const SimTK::State& s);
	void constructDescription();
	void constructColumnLabels();
	void setupStorage();

//=============================================================================
}; // END of class CustomAnalysis
}; //namespace
//=============================================================================
//=============================================================================

#endif // #ifndef OPENSIM_ANALYSIS_PLUGIN_TEMPLATE_H_
