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
         *
         * @tparam Derived The derived class implementing the MotorController interface.
         * @tparam MotorDriverType The type of the motor driver.
         */
        template <typename Derived, typename MotorDriverType>
        class MotorControllerBase
        {
        public:
            /**
             * @brief Constructor for creating a Motor Controller object.
             *
             * @param motor_driver A reference to the MotorDriver object that controls the motor.
             */
            MotorControllerBase(MotorDriverType& motor_driver) : motor_driver_(motor_driver) {}

            /**
             * @brief Set the desired rotation speed of the motor.
             *
             * @param target The desired rotation speed for the motor.
             */
            void set_target(double target) { static_cast<Derived*>(this)->set_target(target); }

            /**
             * @brief Get the current rotation speed of the motor.
             *
             * @return double The current rotation speed of the motor.
             */
            double get_measurement() const { return static_cast<const Derived*>(this)->get_measurement(); }

        protected:
            MotorDriverType& motor_driver_;
        };

    } // namespace motor_control
} // namespace roboost

#endif // MOTOR_CONTROLLER_HPP
