#include <gz/msgs/double.pb.h>

#include <string>
#include <unordered_map>
#include <utility>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/SensorFactory.hh>

#include <sdf/Sensor.hh>

#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/common/Console.hh>

#include "MyOdometerSensor.hh"
#include "MyOdometerSystem.hh"

using namespace custom;


void  MyOdometerSystem::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
               ignition::gazebo::EntityComponentManager &_ecm) {
    
    _ecm.EachNew<ignition::gazebo::components::CustomSensor,
               ignition::gazebo::components::ParentEntity>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *_custom,
        const ignition::gazebo::components::ParentEntity *_parent)->bool
      {
        // Get sensor's scoped name without the world
        auto sensorScopedName = ignition::gazebo::removeParentScope(
            ignition::gazebo::scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _custom->Data();
        data.SetName(sensorScopedName);

        // Default to scoped name as topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/my_odometer";
          data.SetTopic(topic);
        }

        ignition::sensors::SensorFactory sensorFactory;
        auto sensor = sensorFactory.CreateSensor<custom::MyOdometerSensor>(data);
        if (nullptr == sensor)
        {
          ignerr << "Failed to create my_odometer [" << sensorScopedName << "]"
                 << std::endl;
          return false;
        }

        // Set sensor parent
        auto parentName = _ecm.Component<gz::sim::components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // Set topic on Gazebo
        _ecm.CreateComponent(_entity,
            ignition::gazebo::components::SensorTopic(sensor->Topic()));

        // Keep track of this sensor
        this->entitySensorMap.insert(std::make_pair(_entity,
            std::move(sensor)));

        return true;
      });

}

// Documentation inherited.
// During PostUpdate, update the known sensors and publish their data.
// Also remove sensors that have been deleted.

void  MyOdometerSystem::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) {

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    for (auto &[entity, sensor] : this->entitySensorMap)
    {
      sensor->NewPosition(ignition::gazebo::worldPose(entity, _ecm).Pos());
      sensor->Update(_info.simTime);
    }
  }

  this->RemoveSensorEntities(_ecm);


}

/// \brief Remove custom sensors if their entities have been removed from
/// simulation.
/// \param[in] _ecm Immutable reference to ECM.

void  MyOdometerSystem::RemoveSensorEntities(
    const ignition::gazebo::EntityComponentManager &_ecm){
    
    _ecm.EachRemoved<ignition::gazebo::components::CustomSensor>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *)->bool
      {
        if (this->entitySensorMap.erase(_entity) == 0)
        {
          ignerr << "Internal error, missing myodometer for entity ["
                         << _entity << "]" << std::endl;
        }
        return true;
      });

}
IGNITION_ADD_PLUGIN(MyOdometerSystem, ignition::gazebo::System,
  MyOdometerSystem::ISystemPreUpdate,
  MyOdometerSystem::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(MyOdometerSystem, "custom::MyOdometerSystem")
