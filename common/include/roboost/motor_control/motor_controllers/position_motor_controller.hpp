/**
 * @file position_motor_controller.hpp
 * @brief Definition of PositionController, which sets the control output directly to the motor driver with encoder feedback and PID control.
 * @version 0.2
 * @date 2024-05-17
 */

#ifndef POSITION_MOTOR_CONTROLLER_H
#define POSITION_MOTOR_CONTROLLER_H

#include <algorithm>
#include <cmath>
#include <roboost/motor_control/encoders/encoder.hpp>
#include <roboost/motor_control/motor_controllers/motor_controller.hpp>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/filters.hpp>

namespace roboost
{
    namespace motor_control
    {
        /**
         * @brief Implementation of PositionController, which sets the control output directly to the motor driver with encoder feedback and PID control.
         */
        class PositionController : public MotorControllerBase
        {
        public:
            /**
             * @brief Construct a new PositionController object.
             *
             * @param motor_driver The motor driver to control.
             * @param encoder The encoder to read the position from.
             * @param pid_controller The PID controller to use.
             * @param input_filter The filter to use for the input.
             * @param output_filter The filter to use for the output.
             * @param rate_limiting_filter The rate limiting filter to use.
             * @param deadband_threshold The deadband threshold.
             * @param minimum_output The minimum output.
             */
            PositionController(MotorDriverBase& motor_driver, EncoderBase& encoder, controllers::PIDController<float>& pid_controller, filters::FilterBase<float>& input_filter,
                               filters::FilterBase<float>& output_filter, filters::FilterBase<float>& rate_limiting_filter, float deadband_threshold, float minimum_output)
                : MotorControllerBase(motor_driver), encoder_(encoder), pid_(pid_controller), input_filter_(input_filter), output_filter_(output_filter), rate_limiting_filter_(rate_limiting_filter),
                  deadband_threshold_(deadband_threshold), minimum_output_(minimum_output), current_setpoint_(0.0f)
            {
            }

            /**
             * @brief Set the position of the motor.
             *
             * @param desired_angle The desired position in radians.
             */
            void update(float desired_angle) override
            {
                encoder_.update();
                float input = encoder_.ticks_to_radians(encoder_.get_position());
                input = input_filter_.update(input);

                current_setpoint_ = rate_limiting_filter_.update(desired_angle, input);

                float output = pid_.update(current_setpoint_, input);
                output = output_filter_.update(output);

                // Adjust output for deadband and minimum output
                if (fabs(output) < deadband_threshold_)
                {
                    output = 0.0f;
                }
                else if (fabs(output) < minimum_output_)
                {
                    output = minimum_output_ * (output / fabs(output));
                }

                // map output (-1 to 1) to PWM value (-PWM_RESOLUTION to PWM_RESOLUTION)
                constexpr uint16_t pwm_factor = 1 << PWM_RESOLUTION;
                int32_t control_value = static_cast<int32_t>(output * pwm_factor);

                motor_driver_.set_motor_control(control_value);
            }

            /**
             * @brief Get the position of the motor.
             *
             * @return float The position in radians.
             */
            float get_measurement() const override { return encoder_.ticks_to_radians(encoder_.get_position()); }

            float get_setpoint() const { return current_setpoint_; }

        private:
            EncoderBase& encoder_;
            controllers::PIDController<float>& pid_;
            filters::FilterBase<float>& input_filter_;
            filters::FilterBase<float>& output_filter_;
            filters::FilterBase<float>& rate_limiting_filter_;
            float deadband_threshold_;
            float minimum_output_;
            float current_setpoint_;
        };

    } // namespace motor_control
} // namespace roboost

#endif // POSITION_MOTOR_CONTROLLER_H
