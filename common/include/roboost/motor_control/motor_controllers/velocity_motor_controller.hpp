/**
 * @file velocity_motor_controller.hpp
 * @brief Definition of VelocityController, which sets the control output directly to the motor driver with encoder feedback and PID control.
 * @version 0.2
 * @date 2024-05-17
 */

#ifndef VELOCITY_MOTOR_CONTROLLER_HPP
#define VELOCITY_MOTOR_CONTROLLER_HPP

#include <roboost/motor_control/encoders/encoder.hpp>
#include <roboost/motor_control/motor_controllers/motor_controller.hpp>
#include <roboost/utils/callback_scheduler.hpp>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/estimators.hpp>
#include <roboost/utils/filters.hpp>

namespace roboost
{
    namespace motor_control
    {
        /**
         * @brief Implementation of VelocityController, which sets the control output directly to the motor driver with encoder feedback and PID control.
         */
        class VelocityController : public MotorControllerBase
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
             * @param estimator The velocity estimator to use.
             * @param deadband_threshold The deadband threshold.
             * @param minimum_output The minimum output.
             */
            VelocityController(MotorDriverBase& motor_driver, EncoderBase& encoder, controllers::PIDController<float>& pid_controller, filters::FilterBase<float>& input_filter,
                               filters::FilterBase<float>& output_filter, Estimator& estimator, float deadband_threshold, float minimum_output)
                : MotorControllerBase(motor_driver), encoder_(encoder), pid_(pid_controller), input_filter_(input_filter), output_filter_(output_filter), estimator_(estimator),
                  deadband_threshold_(deadband_threshold), minimum_output_(minimum_output), current_setpoint_(0.0f)
            {
            }

            /**
             * @brief Set the rotation speed of the motor.
             *
             * @param desired_rotation_speed The desired rotation speed in rad/s.
             */
            void update(float desired_rotation_speed) override
            {
                encoder_.update();
                float input = encoder_.get_velocity_radians_per_second();
                input = input_filter_.update(input);
                input = estimator_.update(input);

                current_setpoint_ = desired_rotation_speed;

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

                int32_t control_value = static_cast<int32_t>(output * (1 << PWM_RESOLUTION));
                motor_driver_.set_motor_control(control_value);
            }

            /**
             * @brief Get the rotation speed of the motor.
             *
             * @return float The rotation speed in rad/s.
             */
            float get_measurement() const override { return estimator_.update(encoder_.get_velocity_radians_per_second()); }

            float get_setpoint() const { return current_setpoint_; }

        private:
            EncoderBase& encoder_;
            controllers::PIDController<float>& pid_;
            filters::FilterBase<float>& input_filter_;
            filters::FilterBase<float>& output_filter_;
            Estimator& estimator_;
            float deadband_threshold_;
            float minimum_output_;
            float current_setpoint_;
        };

    } // namespace motor_control
} // namespace roboost

#endif // VELOCITY_MOTOR_CONTROLLER_HPP
