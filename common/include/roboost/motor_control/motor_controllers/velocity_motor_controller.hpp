/**
 * @file velocity_motor_controller.hpp
 * @brief Definition of MotorController, which sets the control output directly to the motor driver with encoder feedback and PID control.
 * @version 0.2
 * @date 2024-05-17
 */

#ifndef VELOCITY_MOTOR_CONTROLLER_HPP
#define VELOCITY_MOTOR_CONTROLLER_HPP

#include <roboost/motor_control/encoders/encoder.hpp>
#include <roboost/motor_control/motor_controllers/motor_controller.hpp>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/filters.hpp>

namespace roboost
{
    namespace motor_control
    {
        // TOOD: Move this to a more appropriate location
        inline int32_t fast_map(double x, double in_min, double in_max, int32_t out_min, int32_t out_max) { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

        /**
         * @brief Implementation of MotorController, which sets the control output directly to the motor driver with encoder feedback and PID control.
         *
         * @tparam MotorDriverType The type of the motor driver.
         * @tparam EncoderType The type of the encoder.
         * @tparam ControllerType The type of the PID controller.
         * @tparam InputFilterType The type of the input filter.
         * @tparam OutputFilterType The type of the output filter.
         * @tparam RateLimitingFilterType The type of the rate limiting filter.
         */
        template <typename MotorDriverType, typename EncoderType, typename ControllerType, typename InputFilterType, typename OutputFilterType, typename RateLimitingFilterType>
        class VelocityController
            : public MotorControllerBase<VelocityController<MotorDriverType, EncoderType, ControllerType, InputFilterType, OutputFilterType, RateLimitingFilterType>, MotorDriverType>
        {
        public:
            /**
             * @brief Construct a new VelocityController object.
             *
             * @param motor_driver The motor driver to control.
             * @param encoder The encoder to read the rotation speed from.
             * @param pid_controller The PID controller to use.
             * @param input_filter The filter to use for the input.
             * @param output_filter The filter to use for the output.
             * @param rate_limiting_filter The rate limiting filter to use.
             * @param deadband_threshold The deadband threshold.
             * @param minimum_output The minimum output.
             */
            VelocityController(MotorDriverType& motor_driver, EncoderType& encoder, ControllerType& pid_controller, InputFilterType& input_filter, OutputFilterType& output_filter,
                               RateLimitingFilterType& rate_limiting_filter, double deadband_threshold, double minimum_output)
                : MotorControllerBase<VelocityController<MotorDriverType, EncoderType, ControllerType, InputFilterType, OutputFilterType, RateLimitingFilterType>, MotorDriverType>(motor_driver),
                  encoder_(encoder), pid_(pid_controller), input_filter_(input_filter), output_filter_(output_filter), rate_limiting_filter_(rate_limiting_filter),
                  deadband_threshold_(deadband_threshold), minimum_output_(minimum_output), current_setpoint_(0.0)
            {
            }

            /**
             * @brief Set the rotation speed of the motor.
             *
             * @param desired_rotation_speed The desired rotation speed in rad/s.
             */
            void set_target(double desired_rotation_speed)
            {
                encoder_.update();
                double input = encoder_.ticks_to_radians_per_second(encoder_.get_velocity());
                input = input_filter_.update(input);

                current_setpoint_ = rate_limiting_filter_.update(desired_rotation_speed, input);

                double output = pid_.update(current_setpoint_, input);
                output = output_filter_.update(output);

                // Adjust output for deadband and minimum output
                if (fabs(output) < deadband_threshold_)
                {
                    output = 0.0;
                }
                else if (fabs(output) < minimum_output_)
                {
                    output = minimum_output_ * (output / fabs(output));
                }

                // TODO: Improve this mapping
                int32_t motor_control_value = fast_map(output, -1.0, 1.0, -PWM_RESOLUTION, PWM_RESOLUTION);

                this->motor_driver_.set_motor_control(motor_control_value);
            }

            /**
             * @brief Get the rotation speed of the motor.
             *
             * @return double The rotation speed in rad/s.
             */
            double get_measurement() const { return encoder_.ticks_to_radians_per_second(encoder_.get_velocity()); }

            double get_setpoint() const { return current_setpoint_; }

        private:
            EncoderType& encoder_;
            ControllerType& pid_;
            InputFilterType& input_filter_;
            OutputFilterType& output_filter_;
            RateLimitingFilterType& rate_limiting_filter_;
            double deadband_threshold_;
            double minimum_output_;
            double current_setpoint_;
        };

    } // namespace motor_control
} // namespace roboost

#endif // VELOCITY_MOTOR_CONTROLLER_HPP
