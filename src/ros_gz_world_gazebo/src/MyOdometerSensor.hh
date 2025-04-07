#ifndef MYODOMETERSENSOR_HH_
#define MYODOMETERSENSOR_HH_

#include <gz/transport/Node.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <sdf/Sensor.hh>

namespace custom
{
    // How to know the namespace ?? I search for built in sensors
    //  headerfiles and look at the dependencies and the parents classes e.g. AltimeterSensor.hh
    class MyOdometerSensor : public ignition::sensors::Sensor
    {
    public:
        /// \brief Load the sensor based on data from an sdf::Sensor object.
        /// \param[in] _sdf SDF <sensor> or <plugin> inside of <sensor>
        /// \return true if loading was successful
        virtual bool Load(const sdf::Sensor &_sdf) override;
        // ?
        virtual bool Update(
            const std::chrono::steady_clock::duration &_now) override;
        /// \brief Set the current postiion of the robot, so the odometer can
        /// calculate the distance travelled.
        /// \param[in] _pos Current position in world coordinates.
        void NewPosition(const ignition::math::Vector3d &_pos);
        /// \brief Get the latest world postiion of the robot.
        /// \return The latest position given to the odometer.
        const ignition::math::Vector3d &Position() const;
    private:
        /// \brief Previous position of the robot.
        ignition::math::Vector3d prevPos{std::nan(""), std::nan(""),std::nan("")};
        /// \brief Latest total distance.
        double totalDistance{0.0};

        /// \brief Noise that will be applied to the sensor data
        ignition::sensors::NoisePtr noise{nullptr};

        /// \brief Node for communication
        ignition::transport::Node node;

        /// \brief Publishes sensor data
        ignition::transport::Node::Publisher pub;

        
    };

}

#endif