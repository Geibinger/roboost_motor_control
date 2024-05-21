/**
 * @file motor_controller.hpp
 * @brief Defines the MotorController class, which provides an interface for controlling motors.
 * @version 0.2
 * @date 2024-05-17
 */

#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <roboost/motor_control/motor_drivers/motor_driver.hpp>

namespace roboost
{
    namespace motor_control
    {
        /**
         * @brief Abstract base class for controlling motors.
         */
        class MotorControllerBase
        {
        public:
            /**
             * @brief Constructor for creating a Motor Controller object.
             *
             * @param motor_driver A reference to the MotorDriver object that controls the motor.
             */
            MotorControllerBase(MotorDriverBase& motor_driver) : motor_driver_(motor_driver) {}
            virtual ~MotorControllerBase() = default;

            virtual void update(float target) = 0;

            virtual float get_measurement() const = 0;

        protected:
            MotorDriverBase& motor_driver_;
        };

    } // namespace motor_control
} // namespace roboost

#endif // MOTOR_CONTROLLER_HPP
