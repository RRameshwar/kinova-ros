#ifndef _GAZEBO_INDEX_CONTACT_PLUGIN_HH_
#define _GAZEBO_INDEX_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"


namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class IndexContactPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: IndexContactPlugin();

    /// \brief Destructor.
    public: virtual ~IndexContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

    private:
      ros::NodeHandle n;
      ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("haptic_pointer", 1000);
  };
}
#endif
