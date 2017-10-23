#include "addKneeContacts.h"

void addKneeContactGeometries(Model& model, bool left_knee){
	string LorR = "r";
	if (left_knee) LorR = "l";

	addContactGeometry(model, "meniscus_lat_" + LorR, "meniscus_lat_" + LorR + ".obj");
	addContactGeometry(model, "meniscus_med_" + LorR, "meniscus_med_" + LorR + ".obj");
	addContactGeometry(model, "femur_lat_" + LorR, "femur_lat_" + LorR + ".obj");
	addContactGeometry(model, "femur_med_" + LorR, "femur_med_" + LorR + ".obj");
	addContactGeometry(model, "tibia_upper_" + LorR, "tibia_upper_" + LorR + ".obj");

	model.buildSystem();
};

void addContactGeometry(Model& model, string bodyName, string objName){
	// create body instance of right lateral meniscus
	OpenSim::Body* body = &model.updBodySet().get(bodyName);

	// create contact mesh objects
	ContactMesh *contactMesh = new ContactMesh( "../resources/geometries/" + objName, 
		SimTK::Vec3(0.0), SimTK::Vec3(0.0), *body, bodyName + "_CM");	

	// Add contact mesh to the model
	model.addContactGeometry(contactMesh);
};

void addEFForces(Model& model, double men_stiff, double men_diss, double men_us, double men_ud, double men_uv,
	double art_stiff, double art_diss, double art_us, double art_ud, double art_uv, bool left_knee)
{
	string LorR = "r";
	if (left_knee) LorR = "l";

	OpenSim::ElasticFoundationForce::ContactParameters *contactParamsLat = new OpenSim::ElasticFoundationForce::ContactParameters(men_stiff, men_diss, men_us, men_ud, men_uv);
	contactParamsLat->addGeometry("femur_lat_" + LorR + "_CM");
	contactParamsLat->addGeometry("meniscus_lat_" + LorR + "_CM");

	OpenSim::ElasticFoundationForce *contactForceLat = new OpenSim::ElasticFoundationForce(contactParamsLat);
	contactForceLat->setTransitionVelocity(0.2);
	contactForceLat->setName("contactForce_femur_lat_meniscii_" + LorR);

	model.addForce(contactForceLat);

	OpenSim::ElasticFoundationForce::ContactParameters *contactParamsMed = new OpenSim::ElasticFoundationForce::ContactParameters(men_stiff, men_diss, men_us, men_ud, men_uv);
	contactParamsMed->addGeometry("femur_med_" + LorR + "_CM");
	contactParamsMed->addGeometry("meniscus_med_" + LorR + "_CM");

	OpenSim::ElasticFoundationForce *contactForceMed = new OpenSim::ElasticFoundationForce(contactParamsMed);
	contactForceMed->setTransitionVelocity(0.2);
	contactForceMed->setName("contactForce_femur_med_meniscii_" + LorR);

	model.addForce(contactForceMed);
	
	OpenSim::ElasticFoundationForce::ContactParameters *contactParamsTibMed = new OpenSim::ElasticFoundationForce::ContactParameters(art_stiff, art_diss, art_us, art_ud, art_uv);
	contactParamsTibMed->addGeometry("tibia_upper_" + LorR + "_CM");
	contactParamsTibMed->addGeometry("femur_med_" + LorR + "_CM");

	OpenSim::ElasticFoundationForce *contactForceTibMed = new OpenSim::ElasticFoundationForce(contactParamsTibMed);
	contactForceTibMed->setTransitionVelocity(0.2);
	contactForceTibMed->setName("femur_med_tibia_" + LorR);

	model.addForce(contactForceTibMed);

	OpenSim::ElasticFoundationForce::ContactParameters *contactParamsTibLat = new OpenSim::ElasticFoundationForce::ContactParameters(art_stiff, art_diss, art_us, art_ud, art_uv);
	contactParamsTibLat->addGeometry("tibia_upper_" + LorR + "_CM");
	contactParamsTibLat->addGeometry("femur_lat_" + LorR + "_CM");

	OpenSim::ElasticFoundationForce *contactForceTibLat = new OpenSim::ElasticFoundationForce(contactParamsTibLat);
	contactForceTibLat->setTransitionVelocity(0.2);
	contactForceTibLat->setName("femur_lat_tibia_" + LorR);

	model.addForce(contactForceTibLat);
}