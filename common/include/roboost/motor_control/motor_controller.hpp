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
         * set_rotation_speed method to set the desired rotation speed of the motor.
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
             * @param desired_rotation_speed The desired rotation speed for the motor.
             *
             * @note This method allows setting the desired rotation speed for the motor
             * controlled by the MotorDriver. The actual behavior of the motor may
             * depend on the implementation of the MotorDriver.
             */
            virtual void set_rotation_speed(float desired_rotation_speed) = 0;

            /**
             * @brief Get the current rotation speed of the motor.
             *
             * @return float
             *
             * @note This method returns the current rotation speed of the motor. The
             * actual behavior of the motor may depend on the implementation of the
             * MotorDriver.
             */
            virtual float get_rotation_speed() = 0;

            /**
             * @brief Set the print debug object
             *
             * @param print_debug Whether to print debug information.
             *
             * @note The debug output is formatted for Teleplot.
             */
            void print_debug(bool print_debug) { print_debug_ = print_debug; }

        protected:
            MotorDriver& motor_driver_;
            bool print_debug_ = false;
        };

    } // namespace motor_control
} // namespace roboost

#endif // MOTOR_CONTROLLER_H