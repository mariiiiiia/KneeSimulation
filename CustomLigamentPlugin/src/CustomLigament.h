#ifndef CUSTOMLIGAMENT_H
#define CUSTOMLIGAMENT_H

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/AbstractTool.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Common/Function.h>
#include "osimPluginDLL.h"

namespace OpenSim {

class GeometryPath;

/**
 * A class implementing a CustomLigament. The path of the CustomLigament is
 * stored in a GeometryPath object.
 */
class OSIMPLUGIN_API CustomLigament : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(CustomLigament, Force);

public:
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/    OpenSim_DECLARE_UNNAMED_PROPERTY(GeometryPath, 
		"the set of points defining the path of the CustomLigament");
    OpenSim_DECLARE_PROPERTY(resting_length, double,
		"resting length of the ligament");
	OpenSim_DECLARE_PROPERTY(stiffness, double,
		"stiffness of the ligament");
	OpenSim_DECLARE_PROPERTY(damping, double,
		"damping of the ligament");
	OpenSim_DECLARE_PROPERTY(el, double,
		"el rigion from non-linear to linear");
    /**@}**/

	CustomLigament();

    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

	// Properties
	const GeometryPath& getGeometryPath() const 
    {   
		return get_GeometryPath(); 
	}

	GeometryPath& updGeometryPath() 
    {   
		return upd_GeometryPath();
	}

	virtual bool hasGeometryPath() const {
		return true;
	}

	virtual double getLength(const SimTK::State& s) const;

	virtual double getRestingLength() const 

    {   
		return get_resting_length(); 
	}

	virtual bool setRestingLength(double aRestingLength);

	const double& getTension(const SimTK::State& s) const;

	const double getDamping() const
	{ 
		return get_damping(); 
	}

	bool setDamping(double c)
	{ 
		set_damping(c);
		return true;
	}

	const double getStiffness() const
	{ 
		return get_stiffness(); 
	}

	bool setStiffnessg(double k)
	{ 
		set_stiffness(k);
		return true;
	}

	const double getEL() const
	{ 
		return get_el(); 
	}

	bool setEL(double e_l)
	{ 
		set_el(e_l);
		return true;
	}

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const;
	virtual void computeForce(
		const SimTK::State& s, 
		SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
		SimTK::Vector& generalizedForces) const;

	//--------------------------------------------------------------------------
	// SCALE
	//--------------------------------------------------------------------------
	virtual void preScale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void scale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void postScale(const SimTK::State& s, const ScaleSet& aScaleSet);


	//--------------------------------------------------------------------------
	// Display
	//--------------------------------------------------------------------------
	virtual const VisibleObject* getDisplayer() const;
	virtual void updateDisplayer(const SimTK::State& s) const;

protected:
    /** Override this method if you would like to calculate a color for use
    when the %CustomLigament's path is displayed in the visualizer. You do not have 
    to invoke the base class ("Super") method, just replace it completely. This
    method will be invoked during realizeDynamics() so the supplied \a state has 
    already been realized through Stage::Velocity and you can access time, 
    position, and velocity dependent quantities. You must \e not attempt to 
    realize the passed-in \a state any further since we are already in the 
    middle of realizing here. Return SimTK::Vec3(SimTK::NaN) if you want to 
    leave the color unchanged (that's what the base class implementation does).

    @param[in] state    
        A SimTK::State already realized through Stage::Velocity. Do not 
        attempt to realize it any further.
    @returns 
        The desired color for the path as an RGB vector with each
        component ranging from 0 to 1, or NaN to indicate that the color
        should not be changed. **/
    virtual SimTK::Vec3 computePathColor(const SimTK::State& state) const;

    // Implement ModelComponent interface.
    /** Extension of parent class method; derived classes may extend further. **/
	/**
	 * Perform some setup functions that happen after the
	 * CustomLigament has been deserialized or copied.
	 *
	 * @param aModel model containing this CustomLigament.
	 */
	void connectToModel(Model& aModel) OVERRIDE_11;

    /** Extension of parent class method; derived classes may extend further.
	allocate and initialize the SimTK state for this CustomLigament.**/
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

    /** See if anyone has an opinion about the path color and change it if so. **/
    void realizeDynamics(const SimTK::State& state) const OVERRIDE_11;

	//Force reporting
	/** 
	 * Methods to query a Force for the value actually applied during simulation
	 * The names of the quantities (column labels) is returned by this first function
	 * getRecordLabels()
	 */
	OpenSim::Array<std::string> getRecordLabels() const {
		OpenSim::Array<std::string> labels("");
		labels.append(getName());
		return labels;
	}
	/**
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	OpenSim::Array<double> getRecordValues(const SimTK::State& state) const {
		OpenSim::Array<double> values(1);
		values.append(getTension(state));
		return values;
	}

	
private:
	/**
	 * Construct and initialize the properties for the CustomLigament.
	 * 
	 * You should give each property a meaningful name and an informative comment.
	 * The name you give each property is the tag that will be used in the XML
	 * file. The comment will appear before the property in the XML file.
	 * In addition, the comments are used for tool tips in the OpenSim GUI.
	 *
	 * All properties are added to the property set. Once added, they can be
	 * read in and written to file.
	 */
	void constructProperties();

	/**
	* Computes force-strain curve
	*/
	double force_strain(double e, double k, double e_l) const
	{
		//e = abs(e);

		if(e < 0)
		{
			return 0;
		}
		else if(e <= 2 * e_l)
		{
			return 0.25 * k * e * e / e_l;
		}
		else
		{
			return k * (e - e_l);
		}
	}

//=============================================================================
};	// END of class CustomLigament
//=============================================================================
//=============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_CustomLigament_H_

	
