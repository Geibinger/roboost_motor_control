/**
 * @file robot_controller.hpp
 * @brief A controller for managing motors and controlling their speeds and positions.
 * @version 0.2
 * @date 2024-05-17
 */

#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <roboost/kinematics/base_kinematics.hpp>
#include <roboost/motor_control/motor_control_manager.hpp>
#include <roboost/motor_control/motor_controllers/motor_controller.hpp>
#include <roboost/motor_control/motor_controllers/position_motor_controller.hpp>
#include <roboost/motor_control/motor_controllers/simple_motor_controller.hpp>
#include <roboost/motor_control/motor_controllers/velocity_motor_controller.hpp>
#include <roboost/utils/logging.hpp>
#include <stdint.h>
#include <vector>

namespace roboost
{
    namespace motor_control
    {
        /**
         * @brief The RobotController class manages a collection of motors and their desired speeds or positions.
         */
        class RobotController
        {
        public:
            /**
             * @brief Construct a new Robot Controller object.
             *
             * @param motor_controllers An initializer list of MotorControllerBase pointers.
             */
            RobotController(std::initializer_list<MotorControllerBase*> motor_controllers) : motor_manager_(motor_controllers), kinematics_(nullptr) {}

            /**
             * @brief Set the desired speed or position for a specific motor.
             *
             * @param motor_index The index of the motor.
             * @param desired_value The desired speed or position value.
             */
            void set_motor_target(const uint8_t motor_index, float desired_value) { motor_manager_.set_motor_target(motor_index, desired_value); }

            /**
             * @brief Set the desired speed or position for all motors.
             *
             * @param desired_value The desired speed or position value.
             */
            void set_all_motor_targets(float desired_value) { motor_manager_.set_all_motor_targets(desired_value); }

            /**
             * @brief Get the desired speed or position of a specific motor.
             *
             * @param motor_index The index of the motor.
             * @return float The desired speed or position value.
             */
            float get_motor_target(const uint8_t motor_index) const { return motor_manager_.get_motor_target(motor_index); }

            /**
             * @brief Get the current measurement of a specific motor.
             *
             * @param motor_index The index of the motor.
             * @return float The current measurement value.
             */
            float get_motor_measurement(const uint8_t motor_index) const { return motor_manager_.get_motor_measurement(motor_index); }

            /**
             * @brief Get the number of motors in the robot.
             *
             * @return uint8_t The number of motors.
             */
            uint8_t get_motor_count() const { return motor_manager_.get_motor_count(); }

            /**
             * @brief Update the motors to set the new desired targets (speed or position).
             */
            void update() { motor_manager_.update(); }

            /**
             * @brief Set the kinematics model for the robot.
             *
             * @param kinematics A pointer to the kinematics model.
             */
            void set_kinematics(kinematics::BaseKinematics* kinematics) { kinematics_ = kinematics; }

            /**
             * @brief Calculate the robot state using the kinematics model and wheel velocities.
             *
             * @param wheel_velocities The wheel velocities.
             * @return A unique pointer to the calculated kinematic state.
             */
            std::unique_ptr<kinematics::BaseKinematicState> calculate_robot_state(const std::vector<float>& wheel_velocities) const
            {
                if (kinematics_)
                {
                    return kinematics_->calculate_robot_state(wheel_velocities);
                }
                return nullptr;
            }

            /**
             * @brief Calculate the joint states (wheel velocities) using the kinematics model and robot state.
             *
             * @param robot_state The kinematic state of the robot.
             * @return A vector of calculated joint states (wheel velocities).
             */
            std::vector<float> calculate_joint_states(const kinematics::BaseKinematicState& robot_state) const
            {
                if (kinematics_)
                {
                    return kinematics_->calculate_joint_states(robot_state);
                }
                return {};
            }

        private:
            MotorControllerManager motor_manager_;
            kinematics::BaseKinematics* kinematics_;
        };

    } // namespace motor_control
} // namespace roboost

#endif // ROBOT_CONTROLLER_HPP
