/**
 * @file simple_motor_controller.hpp
 * @brief Definition of SimpleMotorController, which sets the control output directly to the motor driver without feedback loop.
 * @version 0.2
 * @date 2024-05-17
 */

#ifndef SIMPLE_MOTOR_CONTROLLER_H
#define SIMPLE_MOTOR_CONTROLLER_H

#include <algorithm>
#include <roboost/motor_control/motor_controllers/motor_controller.hpp>

namespace roboost
{
    namespace motor_control
    {
        /**
         * @brief Motor controller without encoder feedback or PID.
         */
        class SimpleMotorController : public MotorControllerBase
        {
        public:
            /**
             * @brief Construct a new Simple Motor Controller object.
             *
             * @param motor_driver Motor driver to be used.
             * @param max_rotation_speed Max rotational speed motor driver can output in rad/sec.
             */
            SimpleMotorController(MotorDriverBase& motor_driver, const float max_rotation_speed)
                : MotorControllerBase(motor_driver), max_rotation_speed_(max_rotation_speed), rotation_speed_setpoint_(0.0f)
            {
            }

            /**
             * @brief Set the rotation speed of the motor.
             *
             * @param desired_rotation_speed desired rotation speed in rad/sec.
             */
            void update(float desired_rotation_speed) override
            {
                float clamped_speed = std::clamp(desired_rotation_speed, -max_rotation_speed_, max_rotation_speed_);
                rotation_speed_setpoint_ = clamped_speed / max_rotation_speed_;
                motor_driver_.set_motor_control(static_cast<int32_t>(rotation_speed_setpoint_));
            }

            /**
             * @brief Get the rotation speed of the motor.
             *
             * @return float rotation speed in rad/sec.
             *
             * @note This is not the actual rotation speed of the motor, but the rotation speed that was set using update.
             */
            float get_measurement() const override { return rotation_speed_setpoint_; }

        private:
            const float max_rotation_speed_;
            float rotation_speed_setpoint_;
        };

    } // namespace motor_control
} // namespace roboost

#endif // SIMPLE_MOTOR_CONTROLLER_H
