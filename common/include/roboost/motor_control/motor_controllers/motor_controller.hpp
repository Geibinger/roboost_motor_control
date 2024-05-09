/**
 * @file motor_controller.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Defines the MotorController class, which provides an interface for
 * controlling motors.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <roboost/motor_control/motor_drivers/motor_driver.hpp>

namespace roboost
{
    namespace motor_control
    {

        /**
         * @brief Abstract base class for controlling motors.
         *
         * @note This class defines an interface for controlling motors using a
         * MotorDriver. Subclasses of MotorController are expected to implement the
         * set_target method to set the desired rotation speed of the motor.
         */
        class MotorController
        {
        public:
            /**
             * @brief Constructor for creating a Motor Controller object.
             *
             * @param motor_driver A reference to the MotorDriver object that controls
             * the motor.
             */
            MotorController(MotorDriver& motor_driver) : motor_driver_(motor_driver) {}

            /**
             * @brief Set the desired rotation speed of the motor.
             *
             * @param target The desired rotation speed for the motor.
             *
             * @note This method allows setting the desired rotation speed for the motor
             * controlled by the MotorDriver. The actual behavior of the motor may
             * depend on the implementation of the MotorDriver.
             */
            virtual void set_target(double target) = 0;

            /**
             * @brief Get the current rotation speed of the motor.
             *
             * @return double The current rotation speed of the motor.
             *
             * @note This method returns the current rotation speed of the motor. The
             * actual behavior of the motor may depend on the implementation of the
             * MotorDriver.
             */
            virtual double get_measurement() const = 0;

        protected:
            MotorDriver& motor_driver_;
        };

    } // namespace motor_control
} // namespace roboost

#endif // MOTOR_CONTROLLER_H