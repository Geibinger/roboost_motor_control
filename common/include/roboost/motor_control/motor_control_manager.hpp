/**
 * @file motor_control_manager.hpp
 * @brief A manager for controlling motors and managing their speeds.
 * @version 0.2
 * @date 2024-05-17
 */

#ifndef MOTOR_CONTROL_MANAGER_HPP
#define MOTOR_CONTROL_MANAGER_HPP

#include <roboost/motor_control/motor_controllers/motor_controller.hpp>
#include <roboost/utils/logging.hpp>
#include <stdint.h>
#include <vector>

namespace roboost
{
    namespace motor_control
    {
        /**
         * @brief The MotorControllerManager class manages a collection of motors and
         *        their desired speeds.
         */
        template <typename MotorControllerType>
        class MotorControllerManager
        {
        public:
            /**
             * @brief Construct a new Motor Controller Manager object.
             *
             * @param motor_controllers An initializer list of MotorController pointers.
             */
            MotorControllerManager(std::initializer_list<MotorControllerType*> motor_controllers)
            {
                for (MotorControllerType* motor_controller : motor_controllers)
                {
                    motor_controllers_.push_back({motor_controller, 0.0f});
                }
            }

            /**
             * @brief Set the desired speed for a specific motor.
             *
             * @param motor_index The index of the motor.
             * @param desired_speed The desired speed value.
             */
            void set_motor_speed(const uint8_t motor_index, int32_t desired_speed)
            {
                if (motor_index < 0 || motor_index >= motor_controllers_.size())
                {
                    // Serial.println("Invalid motor index"); // TODO: Implement logging
                }
                else
                {
                    motor_controllers_[motor_index].second = desired_speed;
                }
            }

            /**
             * @brief Set the desired speed for all motors.
             *
             * @param desired_speed The desired speed value.
             */
            void set_all_motor_speeds(int32_t desired_speed)
            {
                for (auto& pair : motor_controllers_)
                {
                    pair.second = desired_speed;
                }
            }

            /**
             * @brief Get the desired speed of a specific motor.
             *
             * @param motor_index The index of the motor.
             * @return int32_t The desired speed value.
             */
            int32_t get_motor_speed(const uint8_t motor_index) const
            {
                if (motor_index < 0 || motor_index >= motor_controllers_.size())
                {
                    // Serial.println("Invalid motor index"); // TODO: Implement logging
                    return 0.0;
                }
                else
                {
                    return motor_controllers_[motor_index].first->get_measurement();
                }
            }

            /**
             * @brief Get the number of MotorControllers in the manager.
             *
             * @return uint8_t The number of motors.
             */
            uint8_t get_motor_count() const { return motor_controllers_.size(); }

            /**
             * @brief Update the MotorControllers to set the new desired rotational
             * speed.
             */
            void update()
            {
                for (auto& pair : motor_controllers_)
                {
                    // Serial.print(">motor "); // TODO: Implement logging
                    // Serial.print(i);
                    // Serial.print(" setpoint:");
                    // Serial.println(pair.second);
                    // Serial.print(">motor ");
                    // Serial.print(i);
                    // Serial.print(" measured:");
                    // Serial.println(pair.first->get_measurement());

                    pair.first->set_target(pair.second);
                }
            }

            /**
             * @brief Destroy the Motor Controller Manager object and free up the
             * memory.
             */
            ~MotorControllerManager()
            {
                for (auto& pair : motor_controllers_)
                {
                    delete pair.first;
                }
            }

        private:
            std::vector<std::pair<MotorControllerType*, int32_t>> motor_controllers_; // The motor controllers and their desired speeds.
        };

    } // namespace motor_control
} // namespace roboost

#endif // MOTOR_CONTROL_MANAGER_HPP
