
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

#ifndef POSITION_MOTOR_CONTROLLER_H
#define POSITION_MOTOR_CONTROLLER_H

#include <roboost/motor_control/encoders/encoder.hpp>
#include <roboost/motor_control/motor_controllers/motor_controller.hpp>
#include <roboost/utils/controllers.hpp>

namespace roboost
{
    namespace motor_control
    {
        // Struct to hold the configuration parameters for the motor controller

        // TODO: Use this struct
        struct PositionControllerConfig
        {
            double kp;
            double ki;
            double kd;
            double max_integral;
            double max_rate_per_second;
            double update_rate;
            double deadband_threshold;
            double minimum_output;
            double input_filter_cutoff_frequency;
            double output_filter_cutoff_frequency;
            double input_filter_sampling_time;
            double output_filter_sampling_time;
        };

        /**
         * @brief Implementation of MotorController, which sets the control output
         * directy to the motor driver with encoder feedback and PID control.
         *
         */
        class PositionController : public MotorController
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
            PositionController(MotorDriver& motor_driver, Encoder& encoder, roboost::controllers::PIDController& pid, roboost::filters::Filter& input_filter, roboost::filters::Filter& output_filter,
                               roboost::filters::RateLimitingFilter& rate_limiting_filter, double deadband_threshold, double minimum_output);

            /**
             * @brief Set the rotation speed of the motor.
             *
             * @param desired_angle The desired rotation speed in rad/s.
             */
            void set_target(double desired_angle);

            /**
             * @brief Get the rotation speed of the motor.
             *
             * @return float The rotation speed in rad/s.
             */
            double get_measurement() const { return encoder_.get_angle(); }

            double get_setpoint() const { return current_setpoint_; }

        private:
            Encoder& encoder_;
            roboost::controllers::PIDController& pid_;
            roboost::filters::Filter& input_filter_;
            roboost::filters::Filter& output_filter_;
            roboost::filters::RateLimitingFilter rate_limiting_filter_;
            double deadband_threshold_;
            double minimum_output_;
            double current_setpoint_;
        };

    } // namespace motor_control
} // namespace roboost

#endif // POSITION_MOTOR_CONTROLLER_H