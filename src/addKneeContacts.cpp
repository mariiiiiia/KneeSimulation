#include "addKneeContacts.h"

void addKneeContactGeometries(Model& model, bool LeftOrRight){
	string LorR = "r";
	if (LeftOrRight) LorR = "l";

	addContactGeometry(model, "meniscus_lat_" + LorR, "meniscus_lat_" + LorR + ".obj");
	addContactGeometry(model, "meniscus_med_" + LorR, "meniscus_med_" + LorR + ".obj");
	addContactGeometry(model, "femur_lat_" + LorR, "femur_lat_" + LorR + ".obj");
	addContactGeometry(model, "femur_med_" + LorR, "femur_med_" + LorR + ".obj");
	//addContactGeometry(model, "tibia_" + LorR, "tibia_upper_" + LorR + ".obj");

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

void addEFForce(Model& model, double stiff, double diss, double us, double ud, double uv, bool LeftOrRight)
{
	string LorR = "r";
	if (LeftOrRight) LorR = "l";

	OpenSim::ElasticFoundationForce::ContactParameters *contactParamsLat = new OpenSim::ElasticFoundationForce::ContactParameters(stiff, diss, us, ud, uv);
	contactParamsLat->addGeometry("femur_lat_" + LorR + "_CM");
	contactParamsLat->addGeometry("meniscus_lat_" + LorR + "_CM");

	OpenSim::ElasticFoundationForce *contactForceLat = new OpenSim::ElasticFoundationForce(contactParamsLat);
	contactForceLat->setTransitionVelocity(0.2);
	contactForceLat->setName("contactForce_femur_lat_meniscii_" + LorR);

	model.addForce(contactForceLat);

	OpenSim::ElasticFoundationForce::ContactParameters *contactParamsMed = new OpenSim::ElasticFoundationForce::ContactParameters(stiff, diss, us, ud, uv);
	contactParamsMed->addGeometry("femur_med_" + LorR + "_CM");
	contactParamsMed->addGeometry("meniscus_med_" + LorR + "_CM");

	OpenSim::ElasticFoundationForce *contactForceMed = new OpenSim::ElasticFoundationForce(contactParamsMed);
	contactForceMed->setTransitionVelocity(0.2);
	contactForceMed->setName("contactForce_femur_med_meniscii_" + LorR);

	model.addForce(contactForceMed);
}


//
//void KneeAdjustment::add_contact_geometry()
//{
//   ContactMesh* femur_r = new ContactMesh( "data/geometry/femur.obj", Vec3(0), Vec3(0), \
//	   m_model->updBodySet().get("femur_r"), "contact_femur_r");
//   m_model->addContactGeometry(femur_r);
//
//   ContactMesh* tibia_r = new ContactMesh( "data/geometry/tibia.obj", Vec3(0), Vec3(0), \
//      m_model->updBodySet().get("tibia_r"), "contact_tibia_r");
//   m_model->addContactGeometry(tibia_r);
//
//   m_model->buildSystem();
//
//
//   double stiffness       = 1.E6;
//    double dissipation     = 1.0;
//    double staticFriction  = 0.8;
//    double dynamicFriction = 0.4;
//    double viscousFriction = 0.4;
// 
//   OpenSim::ElasticFoundationForce::ContactParameters right_knee_param;
//    right_knee_param.addGeometry("contact_femur_r");
//    right_knee_param.addGeometry("contact_tibia_r");
//   right_knee_param.setStiffness(stiffness);
//   right_knee_param.setDissipation(dissipation);
//   right_knee_param.setStaticFriction(staticFriction);
//   right_knee_param.setDynamicFriction(dynamicFriction);
//   right_knee_param.setViscousFriction(viscousFriction);
//
//   right_knee_param.print("data/out/param.xml");
//
//   OpenSim::ElasticFoundationForce* right_knee_force = new OpenSim::ElasticFoundationForce();
//   right_knee_force->setName("right_knee_contact");
//   right_knee_force->addContactParameters(&right_knee_param);
//    right_knee_force->setTransitionVelocity(0.2);
//   
//   right_knee_force->print("data/out/contact.xml");
//
//    m_model->addForce(right_knee_force);
//   
//   /* OpenSim::HuntCrossleyForce* right_knee_contact = new OpenSim::HuntCrossleyForce(right_knee_param);
//    right_knee_contact->setName("right_knee_contact");
//    right_knee_contact->setTransitionVelocity(0.2);
//   
//    m_model->addForce(right_knee_contact);*/
//
//   /*ContactMesh* femur_l = new ContactMesh(
//      "C:/OpenSim 3.2/Geometry/l_femur.vtp", Vec3(0), Vec3(0),
//      m_model->updBodySet().get("femur_l"), "contact_femur_l");
//   femur_l->setDisplayPreference(4);
//   m_model->addContactGeometry(femur_l);
//
//   ContactMesh* tibia_l = new ContactMesh(
//      "C:/OpenSim 3.2/Geometry/l_tibia.vtp", Vec3(0), Vec3(0),
//      m_model->updBodySet().get("tibia_l"), "contact_tibia_l");
//   tibia_l->setDisplayPreference(4);
//   m_model->addContactGeometry(tibia_l);*/
//}