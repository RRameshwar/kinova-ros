#include "IndexContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(IndexContactPlugin)

/////////////////////////////////////////////////
IndexContactPlugin::IndexContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
IndexContactPlugin::~IndexContactPlugin()
{
}

/////////////////////////////////////////////////
void IndexContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{

  

  ros::spinOnce();

  // Get the parent sensor.
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "IndexContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&IndexContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void IndexContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->GetContacts();

  float sum = 0;

  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
      sum = sum + contacts.contact(i).wrench(0).body_2_wrench().force().x();   
  }


  std_msgs::Int16 msg;

  float force_x;
  
  if(sum == 0){
    force_x = 0;
  }
  
  else{
    force_x = (sum/contacts.contact_size());
  }

  int hap;

  if (force_x == 0){
    hap = 0;
  }
  else{
    hap = force_x*255/130;
  }

  msg.data = abs(hap);
  chatter_pub.publish(msg);

}
