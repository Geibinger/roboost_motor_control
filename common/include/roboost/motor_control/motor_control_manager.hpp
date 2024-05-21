/**
 * @file motor_control_manager.hpp
 * @brief A manager for controlling motors and managing their speeds and positions.
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
         *        their desired speeds or positions.
         */
        class MotorControllerManager
        {
        public:
            /**
             * @brief Construct a new Motor Controller Manager object.
             *
             * @param motor_controllers An initializer list of MotorControllerBase pointers.
             */
            MotorControllerManager(std::initializer_list<MotorControllerBase*> motor_controllers)
            {
                for (MotorControllerBase* motor_controller : motor_controllers)
                {
                    motor_controllers_.emplace_back(motor_controller, 0.0f);
                }
            }

            /**
             * @brief Set the desired speed or position for a specific motor.
             *
             * @param motor_index The index of the motor.
             * @param desired_value The desired speed or position value.
             */
            void set_motor_target(const uint8_t motor_index, float desired_value)
            {
                if (motor_index < motor_controllers_.size())
                {
                    motor_controllers_[motor_index].second = desired_value;
                }
                else
                {
                    // Logging or error handling can be done here.
                }
            }

            /**
             * @brief Set the desired speed or position for all motors.
             *
             * @param desired_value The desired speed or position value.
             */
            void set_all_motor_targets(float desired_value)
            {
                for (auto& pair : motor_controllers_)
                {
                    pair.second = desired_value;
                }
            }

            /**
             * @brief Get the desired speed or position of a specific motor.
             *
             * @param motor_index The index of the motor.
             * @return float The desired speed or position value.
             */
            float get_motor_target(const uint8_t motor_index) const
            {
                if (motor_index < motor_controllers_.size())
                {
                    return motor_controllers_[motor_index].second;
                }
                else
                {
                    // Logging or error handling can be done here.
                    return 0.0f;
                }
            }

            /**
             * @brief Get the current measurement of a specific motor.
             *
             * @param motor_index The index of the motor.
             * @return float The current measurement value.
             */
            float get_motor_measurement(const uint8_t motor_index) const
            {
                if (motor_index < motor_controllers_.size())
                {
                    return motor_controllers_[motor_index].first->get_measurement();
                }
                else
                {
                    // Logging or error handling can be done here.
                    return 0.0f;
                }
            }

            /**
             * @brief Get the number of MotorControllers in the manager.
             *
             * @return uint8_t The number of motors.
             */
            uint8_t get_motor_count() const { return motor_controllers_.size(); }

            /**
             * @brief Update the MotorControllers to set the new desired target (speed or position).
             */
            void update()
            {
                for (auto& pair : motor_controllers_)
                {
                    pair.first->update(pair.second);
                }
            }

            /**
             * @brief Destroy the Motor Controller Manager object and free up the memory.
             */
            ~MotorControllerManager()
            {
                for (auto& pair : motor_controllers_)
                {
                    delete pair.first;
                }
            }

        private:
            std::vector<std::pair<MotorControllerBase*, float>> motor_controllers_; // The motor controllers and their desired targets.
        };

    } // namespace motor_control
} // namespace roboost

#endif // MOTOR_CONTROL_MANAGER_HPP
