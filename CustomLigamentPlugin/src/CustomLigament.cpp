#include "CustomLigament.h"

#include <OpenSim/Simulation/Model/GeometryPath.h>
#include <OpenSim/Simulation/Model/PointForceDirection.h>
#include <OpenSim/Common/SimmSpline.h>

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static const Vec3 DefaultCustomLigamentColor(.9,.9,.9); 

CustomLigament::CustomLigament()
{
	constructProperties();
}

void CustomLigament::constructProperties()
{
	setAuthors("Jim Stanev");
	constructProperty_GeometryPath(GeometryPath());

	constructProperty_resting_length(0.0);
	constructProperty_damping(0.0);
	constructProperty_stiffness(0.0);
	constructProperty_el(0.01);

}

void CustomLigament::connectToModel(Model& aModel)
{
	GeometryPath& path = upd_GeometryPath();
	const double& restingLength = get_resting_length();

    path.setDefaultColor(DefaultCustomLigamentColor);

	// Specify underlying ModelComponents prior to calling 
    // Super::connectToModel() to automatically propagate connectToModel()
    // to subcomponents. Subsequent addToSystem() will also be automatically
	// propagated to subcomponents.
    // TODO: this is awkward; subcomponent API needs to be revisited (sherm)
	includeAsSubComponent(&path);

    //TODO: can't call this at start of override; this is an API bug.
	Super::connectToModel(aModel);

	// _model will be NULL when objects are being registered.
	if (_model == NULL)
		return;

	// Resting length must be greater than 0.0.
	assert(restingLength > 0.0);

	path.setOwner(this);
}

void CustomLigament::realizeDynamics(const SimTK::State& state) const {
    Super::realizeDynamics(state); // Mandatory first line

	if(!isDisabled(state)){
		const SimTK::Vec3 color = computePathColor(state);
		if (!color.isNaN())
			getGeometryPath().setColor(state, color);
	}
}

/**
* This is the CustomLigament base class implementation for choosing the path
* color. Derived classes can override this with something meaningful.
* TODO: should the default attempt to use the CustomLigament tension to control
* colors? Not sure how to scale.
*/
SimTK::Vec3 CustomLigament::computePathColor(const SimTK::State& state) const {
    return SimTK::Vec3(SimTK::NaN);
}

void CustomLigament::addToSystem(SimTK::MultibodySystem& system) const
{
	Super::addToSystem(system);
	// Cache the computed tension and strain of the CustomLigament
	addCacheVariable<double>("tension", 0.0, SimTK::Stage::Velocity);
	addCacheVariable<double>("strain", 0.0, SimTK::Stage::Velocity);
}
 
//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the length of the CustomLigament. This is a convenience function that passes
 * the request on to the CustomLigament path.
 *
 * @return Current length of the CustomLigament path.
 */
double CustomLigament::getLength(const SimTK::State& s) const
{
	return getGeometryPath().getLength(s);
}

//_____________________________________________________________________________
/**
 * Set the resting length.
 *
 * @param aRestingLength The resting length of the CustomLigament.
 * @return Whether the resting length was successfully changed.
 */
bool CustomLigament::setRestingLength(double aRestingLength)
{
	set_resting_length(aRestingLength);
	return true;
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform computations that need to happen before the CustomLigament is scaled.
 * For this object, that entails calculating and storing the
 * length in the current body position.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void CustomLigament::preScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	updGeometryPath().preScale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Scale the CustomLigament.
 *
 * @param aScaleSet XYZ scale factors for the bodies
 * @return Whether or not the CustomLigament was scaled successfully
 */
void CustomLigament::scale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	updGeometryPath().scale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Perform computations that need to happen after the CustomLigament is scaled.
 * For this object, that entails comparing the length before and after scaling,
 * and scaling the resting length a proportional amount.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void CustomLigament::postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	GeometryPath& path          = updGeometryPath();
	double&       restingLength = upd_resting_length();

	path.postScale(s, aScaleSet);

	if (path.getPreScaleLength(s) > 0.0)
	{
		double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);

		// Scale resting length by the same amount as the change in
		// total CustomLigament length (in the current body position).
		restingLength *= scaleFactor;

		path.setPreScaleLength(s, 0.0);
	}
}

const double& CustomLigament::getTension(const SimTK::State& s) const
{
	return getCacheVariable<double>(s, "tension"); 
}


//=============================================================================
// COMPUTATION
//=============================================================================
/**
 * Compute the moment-arm of this muscle about a coordinate.
 */
double CustomLigament::computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const
{
	return getGeometryPath().computeMomentArm(s, aCoord);
}



void CustomLigament::computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	const GeometryPath& path = getGeometryPath();
	const double& restingLength = get_resting_length();
	const double& damping = get_damping();

	double force = 0;

	if (path.getLength(s) <= restingLength){
		setCacheVariable<double>(s, "tension", force);
		return;
	}
	
	// evaluate normalized tendon force length curve
	const double strain_force = force_strain(
		(path.getLength(s) - restingLength) / restingLength, 
		get_stiffness(), 
		get_el());
	//force = f(e) + dumping * lengthening_speed
	force = strain_force + damping * path.getLengtheningSpeed(s);

	setCacheVariable<double>(s, "tension", force);

	OpenSim::Array<PointForceDirection*> PFDs;
	path.getPointForceDirections(s, &PFDs);

	for (int i=0; i < PFDs.getSize(); i++) {
		applyForceToPoint(s, PFDs[i]->body(), PFDs[i]->point(), 
                          force*PFDs[i]->direction(), bodyForces);
	}
	for(int i=0; i < PFDs.getSize(); i++)
		delete PFDs[i];
}

//_____________________________________________________________________________
/**
 * Get the visible object used to represent the CustomLigament.
 */
const VisibleObject* CustomLigament::getDisplayer() const
{ 
	return getGeometryPath().getDisplayer(); 
}

//_____________________________________________________________________________
/**
 * Update the visible object used to represent the CustomLigament.
 */
void CustomLigament::updateDisplayer(const SimTK::State& s) const
{
	getGeometryPath().updateDisplayer(s);
}
