#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/Ligament.h>
#include "CustomAnalysis.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

CustomAnalysis::CustomAnalysis() 
	: Analysis()
{
	setNull();
	constructProperties();
}

/**
 * Default constructor
 */
CustomAnalysis::CustomAnalysis(Model *model, string leg) 
	: Analysis(model), m_leg(leg)
{
    setNull();
    constructProperties();

	// set up
	constructDescription();
	constructColumnLabels();
	setupStorage();
}


/**
 * SetNull()
 */
void CustomAnalysis::setNull()
{
    m_initial_position = Vec3(0);
	//m_initial_rotation = Vec3(0);
	m_leg = "r";
}

/*
 * Connect properties to local pointers.
 */
void CustomAnalysis::constructProperties()
{
  /*  Array<string> defaultBodyNames;
    defaultBodyNames.append("all");
    constructProperty_body_names(defaultBodyNames);*/

    // Here are some examples of other constructing other scalar property types.
    // Uncomment them as you need them.
    // ------------------------------------------------------
    //constructProperty_string_property("defaultString");
    //constructProperty_int_property(10);
    //constructProperty_bool_property(true);
    //constructProperty_double_property(1.5);
}

/**
 * Construct a description for the body kinematics files.
 */
void CustomAnalysis::constructDescription()
{
	string descrip;

	descrip = "\nThis file contains tibia displacement and acting force.\n";
	descrip += "\nUnits are S.I. units (seconds, meters, Newtons, ...)";
	if(getInDegrees()) {
		descrip += "\nAngles are in degrees.";
	} else {
		descrip += "\nAngles are in radians.";
	}
	descrip += "\n\n";

	setDescription(descrip);
}

/**
 * Construct column labels for the output results.
 *
 * For analyses that run during a simulation, the first column is almost
 * always time.  For the purpose of example, the code below adds labels
 * appropriate for recording the translation and orientation of each
 * body in the model.
 *
 * This method needs to be called as necessary to update the column labels.
 */
void CustomAnalysis::constructColumnLabels()
{
	if(_model==NULL) return;

	Array<string> labels;
	labels.append("time");
	//labels.append("displacement");
	labels.append("APT");
	labels.append("SIT");
	//labels.append("LMT");

	//labels.append("APT_F");
	//labels.append("SIT_F");

	labels.append("knee_angle");

#ifndef OSIMPLUGIN_API
#ifdef SAVE_FORCE
	labels.append("force");
#endif
#endif
	for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	{
		if(_model->updForceSet()[i].getName().substr(2, 2) == "CL")
		{
			labels.append(_model->updForceSet()[i].getName() + "_force");
		}
	}

	for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	{
		if(_model->updForceSet()[i].getName().substr(2, 2) == "CL")
		{
			labels.append(_model->updForceSet()[i].getName() + "_length");
		}
	}

	for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	{
		if(_model->updForceSet()[i].getName().substr(2, 2) == "CL")
		{
			labels.append(_model->updForceSet()[i].getName() + "_strain");
		}
	}
	
	//for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	//{
	//	if(_model->updForceSet()[i].getName().substr(0, 6) == "femur_")
	//	{
	//		labels.append(_model->updForceSet()[i].getRecordLabels());
	//	}
	//}

	//string contact_label = "";
	//for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	//{
	//	std::string name = _model->updForceSet()[i].getName();
	//	if(name.substr(0, 20) == "femur_lat_meniscii_r")
	//	{
	//		Array<string> contact_force_array = _model->updForceSet().get(name).getRecordLabels();
	//		// get force.x force.y force.z
	//		for (int j=0; j<3; j++)
	//			contact_label += contact_force_array[j];
	//	}
	//}
	//labels.append(contact_label);

	//contact_label = "";
	//for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	//{
	//	std::string name = _model->updForceSet()[i].getName();
	//	if(name.substr(0, 20) == "femur_med_meniscii_r")
	//	{
	//		Array<string> contact_force_array = _model->updForceSet().get(name).getRecordLabels();
	//		// get force.x force.y force.z
	//		for (int j=0; j<3; j++)
	//			contact_label += contact_force_array[j];
	//	}
	//}
	//labels.append(contact_label);

	//contact_label = "";
	//for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	//{
	//	std::string name = _model->updForceSet()[i].getName();
	//	if(name.substr(0, 17) == "femur_lat_tibia_r")
	//	{
	//		Array<string> contact_force_array = _model->updForceSet().get(name).getRecordLabels();
	//		// get force.x force.y force.z
	//		for (int j=0; j<3; j++)
	//			contact_label += contact_force_array[j];
	//	}
	//}
	//labels.append(contact_label);

	//contact_label = "";
	//for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	//{
	//	std::string name = _model->updForceSet()[i].getName();
	//	if(name.substr(0, 17) == "femur_med_tibia_r")
	//	{
	//		Array<string> contact_force_array = _model->updForceSet().get(name).getRecordLabels();
	//		// get force.x force.y force.z
	//		for (int j=0; j<3; j++)
	//			contact_label += contact_force_array[j];
	//	}
	//}
	//labels.append(contact_label);

	labels.append( "femur_lat_meniscii_force_r");
	labels.append( "femur_lat_meniscii_torque_r");
	labels.append( "femur_med_meniscii_force_r");
	labels.append( "femur_med_meniscii_torque_r");
	//labels.append( "meniscii_femur_med_force_r");
	labels.append( "femur_lat_tibia_force_r");
	labels.append( "femur_lat_tibia_torque_r");
	labels.append( "femur_med_tibia_force_r");
	labels.append( "femur_med_tibia_torque_r");
	
	setColumnLabels(labels);
}

/**
 * Set up storage objects.
 *
 * In general, the storage objects in your analysis are used to record
 * the results of your analysis and write them to file.  You will often
 * have a number of storage objects, each for recording a different
 * kind of result.
 */
void CustomAnalysis::setupStorage()
{
	m_storage.reset(0);
	m_storage.setName("custom-analysis");
	m_storage.setDescription(getDescription());
	m_storage.setColumnLabels(getColumnLabels());
}


/**
 * Set the model for which this analysis is to be run.
 *
 * Sometimes the model on which an analysis should be run is not available
 * at the time an analysis is created.  Or, you might want to change the
 * model.  This method is used to set the model on which the analysis is
 * to be run.
 *
 * @param aModel Model pointer
 */
void CustomAnalysis::setModel(Model& aModel)
{
	// SET THE MODEL IN THE BASE CLASS
	Super::setModel(aModel);

	// UPDATE VARIABLES IN THIS CLASS
	constructDescription();
	constructColumnLabels();
	setupStorage();
}

/**
 * Compute and record the results.
 *
 * This method, for the purpose of example, records the position and
 * orientation of each body in the model.  You will need to customize it
 * to perform your analysis.
 *
 * @param aT Current time in the simulation.
 * @param aX Current values of the controls.
 * @param aY Current values of the states: includes generalized coords and speeds
 */
int CustomAnalysis::record(const SimTK::State& s)
{
	_model->getMultibodySystem().realize(s, SimTK::Stage::Dynamics);

	const Body& tibia = _model->getBodySet().get("tibia_" + m_leg);
	SimbodyEngine& engine = _model->updSimbodyEngine();

	//get displacement, a-t, s-i, l-m translations
	Transform tibia_GT = engine.getTransform(s, tibia);

	if(s.getTime() == 0)
	{
		m_initial_position = tibia_GT.p();
		//m_initial_rotation = tibia_GT.R();
	}

	Vec3 position = tibia_GT.p();
	Vec3 diff = position - m_initial_position;

	//Vec3 rotation = tibia_GT.R();

	//Real displacement = std::sqrt(~diff * diff);

	//if(diff[0] < 0)
	//{
	//	displacement *= -1;//work only for Lachman test
	//}

	Real a_p_t = ~diff * tibia_GT.x();
	Real s_i_t = ~diff * tibia_GT.y();
	//Real l_m_t = ~diff * tibia_GT.z();

	double knee_angle_rad = _model->getCoordinateSet().get("knee_angle_r").getValue(s);


#ifndef OSIMPLUGIN_API
#ifdef SAVE_FORCE
	//get external force
	const Force& force = _model->getForceSet().get("external_force");

	Array<double> external_force = force.getRecordValues(s);
	Vec3 f_t(external_force[0], external_force[1], external_force[2]);
	Real force_magnitude = std::sqrt(~f_t * f_t);
#endif
#endif
	//append storage
	Array<double> data;
	//data.append(displacement);

	data.append(a_p_t);
	data.append(s_i_t);
	//data.append(l_m_t);

	//data.append(position[0]);
	//data.append(position[1]);

	data.append(-57.296 * knee_angle_rad); //converted to degree

#ifndef OSIMPLUGIN_API
#ifdef SAVE_FORCE
	data.append(force_magnitude);
#endif
#endif
	//add ligament forces
	for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	{
		std::string name = _model->updForceSet()[i].getName();
		if(name.substr(2, 2) == "CL")
		{
			Ligament& ligament = (Ligament &) _model->updForceSet().get(name);
			double force = ligament.getTension(s);
			data.append(force);
		}
	}

	//add ligament length
	for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	{
		std::string name = _model->updForceSet()[i].getName();
		if(name.substr(2, 2) == "CL")
		{
			Ligament& ligament = (Ligament &) _model->updForceSet().get(name);
			double length = ligament.getLength(s);
			data.append(length);
		}
	}

	//add ligament strain
	for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	{
		std::string name = _model->updForceSet()[i].getName();
		if(name.substr(2, 2) == "CL")
		{
			Ligament& ligament = (Ligament &) _model->updForceSet().get(name);
			double strain = 
				(ligament.getLength(s) - ligament.getRestingLength()) / ligament.getRestingLength();
			data.append(strain);
		}
	}

	//add contact forces
	//for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	//{
	//	std::string name = _model->updForceSet()[i].getName();
	//	if(name.substr(0, 6) == "femur_")
	//	{
	//		Array<double> contact_force = _model->updForceSet().get(name).getRecordValues(s);
	//		data.append(contact_force);
	//	}
	//}

	// sum contact force between femur lateral and meniscus lateral
	Real femur_lat_meniscii_force = 0.0;
	Real femur_lat_meniscii_torque = 0.0;
	for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	{
		std::string name = _model->updForceSet()[i].getName();
		if(name.substr(0, 20) == "femur_lat_meniscii_r")
		{
			Array<double> contact_array = _model->updForceSet().get(name).getRecordValues(s);
			// get force.x force.y force.z
			for (int j=0; j<3; j++)
				femur_lat_meniscii_force += contact_array[j]*contact_array[j];
			// get torque.x torque.y torque.z
			for (int t=3; t<6; t++)
				femur_lat_meniscii_torque += contact_array[t]*contact_array[t];
		}
	}
	data.append( std::sqrt( femur_lat_meniscii_force));
	data.append( std::sqrt( femur_lat_meniscii_torque));
	
	// sum contact force between femur medial and meniscus medial
	Real femur_med_meniscii_force = 0.0;
	//Real meniscii_femur_med_force = 0.0;
	Real femur_med_meniscii_torque = 0.0;
	for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	{
		std::string name = _model->updForceSet()[i].getName();
		if(name.substr(0, 20) == "femur_med_meniscii_r")
		{
			Array<double> contact_array = _model->updForceSet().get(name).getRecordValues(s);
			// get force.x force.y force.z
			for (int j=0; j<3; j++)
				femur_med_meniscii_force += contact_array[j]*contact_array[j];
			//for (int k=6; k<9; k++)
			//	meniscii_femur_med_force += contact_force_array[k]*contact_force_array[k];
			// get torque.x torque.y torque.z
			for (int t=3; t<6; t++)
				femur_med_meniscii_torque += contact_array[t]*contact_array[t];
		}
	}
	data.append( std::sqrt( femur_med_meniscii_force));
	//data.append( std::sqrt( meniscii_femur_med_force));
	data.append( std::sqrt( femur_med_meniscii_torque));

	// sum contact force between femur medial and tibia
	Real femur_lat_tibia_force = 0.0;
	Real femur_lat_tibia_torque = 0.0;
	for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	{
		std::string name = _model->updForceSet()[i].getName();
		if(name.substr(0, 17) == "femur_lat_tibia_r")
		{
			Array<double> contact_array = _model->updForceSet().get(name).getRecordValues(s);
			// get force.x force.y force.z
			for (int j=0; j<3; j++)
				femur_lat_tibia_force += contact_array[j]*contact_array[j];
			// get torque.x torque.y torque.z
			for (int t=3; t<6; t++)
				femur_lat_tibia_torque += contact_array[t]*contact_array[t];
		}
	}
	data.append( std::sqrt( femur_lat_tibia_force));
	data.append( std::sqrt( femur_lat_tibia_torque));

	// sum contact force between femur medial and tibia
	Real femur_med_tibia_force = 0.0;
	Real femur_med_tibia_torque = 0.0;
	for(int i = 0 ; i < _model->updForceSet().getSize() ; i++)
	{
		std::string name = _model->updForceSet()[i].getName();
		if(name.substr(0, 17) == "femur_med_tibia_r")
		{
			Array<double> contact_array = _model->updForceSet().get(name).getRecordValues(s);
			// get force.x force.y force.z
			for (int j=0; j<3; j++)
				femur_med_tibia_force += contact_array[j]*contact_array[j];
			// get torque.x torque.y torque.z
			for (int t=3; t<6; t++)
				femur_med_tibia_torque += contact_array[t]*contact_array[t];
		}
	}
	data.append( std::sqrt( femur_med_tibia_force));
	data.append( std::sqrt( femur_med_tibia_torque));

	m_storage.append(s.getTime(), data.size(), &data[0]);

	return(0);
}

/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration in
 * Model::integBeginCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that will be attempted.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int CustomAnalysis::begin(SimTK::State& s)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	m_storage.reset(s.getTime());

	// RECORD
	int status = 0;
	if(m_storage.getSize()<=0) {
		status = record(s);
	}

	return(status);
}

/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called in
 * Model::integStepCallback(), which has the same argument list.
 *
 * @param aXPrev Controls at the beginining of the current time step.
 * @param aYPrev States at the beginning of the current time step.
 * @param aYPPrev Pseudo states at the beginning of the current time step.
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just taken.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int CustomAnalysis::step(const SimTK::State& s, int stepNumber)
{
	if(!proceed(stepNumber)) return(0);

	record(s);

	return(0);
}

/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * Model::integEndCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int CustomAnalysis::end(SimTK::State& s)
{
	if(!proceed()) return(0);

	record(s);

	return(0);
}

int CustomAnalysis::print(const std::string &path)
{
	m_storage.print(path);

	return(0);
}

void CustomAnalysis::get_displacement_column(Array<double> &force, Array<double> &displacement)
{ 
	m_storage.getDataColumn("displacement", displacement);
#ifndef OSIMPLUGIN_API
#ifdef SAVE_FORCE
	m_storage.getDataColumn("force", force);
#endif
#endif
}

/**
 * Print results.
 * 
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int CustomAnalysis::printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	Storage::printResult(&m_storage, aBaseName + "_" + getName(), aDir, aDT, aExtension);

	return(0);
}
