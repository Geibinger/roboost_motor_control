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

#ifndef SIMPLE_MOTOR_CONTROLLER_H
#define SIMPLE_MOTOR_CONTROLLER_H

#include <roboost/motor_control/motor_controllers/motor_controller.hpp>

namespace roboost
{
    namespace motor_control
    {
        /**
         * @brief Motor controller without encoder feedback or PID
         *
         */
        class SimpleMotorController : public MotorController
        {
        public:
            /**
             * @brief Construct a new Simple Motor Controller object
             *
             * @param motor_driver Motor driver to be used
             * @param max_rotation_speed Max rotational speed motor driver can output in
             * rad/sec
             */
            SimpleMotorController(MotorDriver& motor_driver, const double max_rotation_speed);

            /**
             * @brief Set the rotation speed of the motor
             *
             * @param desired_rotation_speed desired rotation speed in rad/sec
             */
            void set_target(const double desired_rotation_speed);

            /**
             * @brief Get the rotation speed of the motor
             *
             * @return float rotation speed in rad/sec
             *
             * @note This is not the actual rotation speed of the motor, but the
             * rotation speed that was set using set_target.
             */
            double get_measurement() const;

        private:
            const double max_rotation_speed_;
            double rotation_speed_setpoint_;
        };

    } // namespace motor_control
} // namespace roboost

#endif // SIMPLE_MOTOR_CONTROLLER_H
