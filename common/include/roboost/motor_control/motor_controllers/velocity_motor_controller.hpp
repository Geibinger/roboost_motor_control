
/**
 * @file simple_motor_controller.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Definition of MotorController, which sets the control output
 * directy to the motor driver without feedback loop.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef VELOCITY_MOTOR_CONTROLLER_H
#define VELOCITY_MOTOR_CONTROLLER_H

#include <roboost/motor_control/encoders/encoder.hpp>
#include <roboost/motor_control/motor_controllers/motor_controller.hpp>
#include <roboost/utils/controllers.hpp>

namespace roboost
{
    namespace motor_control
    {
        /**
         * @brief Implementation of MotorController, which sets the control output
         * directy to the motor driver with encoder feedback and PID control.
         *
         */
        class VelocityController : public MotorController
        {
        public:
            /**
             * @brief Construct a new VelocityController object
             *
             * @param motor_driver The motor driver to control.
             * @param encoder The encoder to read the rotation speed from.
             * @param pid_controller The PID controller to use.
             * @param input_filter The filter to use for the input.
             * @param output_filter The filter to use for the output.
             */
            VelocityController(MotorDriver& motor_driver, Encoder& encoder, roboost::controllers::PIDController& pid_controller, roboost::filters::Filter& input_filter,
                               roboost::filters::Filter& output_filter);

            /**
             * @brief Set the rotation speed of the motor.
             *
             * @param desired_rotation_speed The desired rotation speed in rad/s.
             */
            void set_target(double desired_rotation_speed);

            /**
             * @brief Get the rotation speed of the motor.
             *
             * @return float The rotation speed in rad/s.
             */
            double get_measurement() const;

        private:
            Encoder& encoder_;
            roboost::controllers::PIDController& pid_;
            roboost::filters::Filter& input_filter_;
            roboost::filters::Filter& output_filter_;
        };

    } // namespace motor_control
} // namespace roboost

#endif // VELOCITY_MOTOR_CONTROLLER_H