#ifndef MYODOMETERSYSTEM_HH_
#define MYODOMETERSYSTEM_HH_

#include <gz/sim/System.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>
namespace custom
{
    //take a look at sim/System.hh, all three classes are defined there  
    class MyOdometerSystem:
        public ignition::gazebo::System,
        public ignition::gazebo::ISystemPreUpdate,
        public ignition::gazebo::ISystemPostUpdate
    {
        // Documentation inherited.
        // During PreUpdate, check for new sensors that were inserted
        // into simulation and create more components as needed.
        public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) final;

        // Documentation inherited.
        // During PostUpdate, update the known sensors and publish their data.
        // Also remove sensors that have been deleted.
        public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
            const gz::sim::EntityComponentManager &_ecm) final;

        /// \brief Remove custom sensors if their entities have been removed from
        /// simulation.
        /// \param[in] _ecm Immutable reference to ECM.
        private: void RemoveSensorEntities(
            const ignition::gazebo::EntityComponentManager &_ecm);

        /// \brief A map of custom entities to their sensors
        private: std::unordered_map<ignition::gazebo::Entity,
            std::shared_ptr<MyOdometerSensor>> entitySensorMap;
    };
}
#endif