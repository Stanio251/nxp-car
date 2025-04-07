
#include <math.h>

#include <gz/msgs/double.pb.h>
#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Util.hh>

#include <sdf/Sensor.hh>

#include "MyOdometerSensor.hh"


using namespace custom;
//////////////////////////////////////////////////
bool MyOdometerSensor::Load(const sdf::Sensor &_sdf)
{
  auto type = ignition::sensors::customType(_sdf);
  if ("my_odometer" != type)
  {
    ignerr << "Trying to load [my_odometer] sensor, but got type ["
           << type << "] instead." << std::endl;
    return false;
  }

  // Load common sensor params
  ignition::sensors::Sensor::Load(_sdf);

  // Advertise topic where data will be published
  this->pub = this->node.Advertise<ignition::msgs::Double>(this->Topic());

  if (!_sdf.Element()->HasElement("ignition:my_odometer")) //look at gz/sensors/Util.hh
  {
    igndbg << "No custom configuration for [" << this->Topic() << "]"
           << std::endl;
    return true;
  }

  // Load custom sensor params
  auto customElem = _sdf.Element()->GetElement("ignition:my_odometer");

  if (!customElem->HasElement("noise"))
  {
    igndbg << "No noise for [" << this->Topic() << "]" << std::endl;
    return true;
  }

  sdf::Noise noiseSdf;// take a look at sdf/Noise.hh-> here is the sdf::Noise class is defined and at sensors/Noise.hh-> we define NewNoiseModel in the class NoiseFactory
  noiseSdf.Load(customElem->GetElement("noise"));
  this->noise = ignition::sensors::NoiseFactory::NewNoiseModel(noiseSdf);
  if (nullptr == this->noise)
  {
    ignerr << "Failed to load noise." << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool MyOdometerSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  ignition::msgs::Double msg;
  *msg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  //this->totalDistance = this->noise->Apply(this->totalDistance);

  msg.set_data(this->totalDistance);

  this->AddSequence(msg.mutable_header());
  this->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
void MyOdometerSensor::NewPosition(const gz::math::Vector3d &_pos)
{
  if (!isnan(this->prevPos.X()))
  {
    this->totalDistance += this->prevPos.Distance(_pos);
  }
  this->prevPos = _pos;
}

//////////////////////////////////////////////////
const ignition::math::Vector3d &MyOdometerSensor::Position() const
{
  return this->prevPos;
}