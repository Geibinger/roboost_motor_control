/**
 * @file position_motor_controller.hpp
 * @brief Definition of MotorController, which sets the control output directly to the motor driver with encoder feedback and PID control.
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
        class PositionController
            : public MotorControllerBase<PositionController<MotorDriverType, EncoderType, ControllerType, InputFilterType, OutputFilterType, RateLimitingFilterType>, MotorDriverType>
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
            PositionController(MotorDriverType& motor_driver, EncoderType& encoder, ControllerType& pid_controller, InputFilterType& input_filter, OutputFilterType& output_filter,
                               RateLimitingFilterType& rate_limiting_filter, double deadband_threshold, double minimum_output)
                : MotorControllerBase<PositionController<MotorDriverType, EncoderType, ControllerType, InputFilterType, OutputFilterType, RateLimitingFilterType>, MotorDriverType>(motor_driver),
                  encoder_(encoder), pid_(pid_controller), input_filter_(input_filter), output_filter_(output_filter), rate_limiting_filter_(rate_limiting_filter),
                  deadband_threshold_(deadband_threshold), minimum_output_(minimum_output), current_setpoint_(0.0)
            {
            }

            /**
             * @brief Set the position of the motor.
             *
             * @param desired_angle The desired position in radians.
             */
            void set_target(double desired_angle)
            {
                encoder_.update();
                double input = encoder_.ticks_to_radians(encoder_.get_position());
                input = input_filter_.update(input);

                current_setpoint_ = rate_limiting_filter_.update(desired_angle, input);

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

                this->motor_driver_.set_motor_control(static_cast<int32_t>(output));
            }

            /**
             * @brief Get the position of the motor.
             *
             * @return double The position in radians.
             */
            double get_measurement() const { return encoder_.ticks_to_radians(encoder_.get_position()); }

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

#endif // POSITION_MOTOR_CONTROLLER_H
