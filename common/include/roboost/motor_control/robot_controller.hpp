/**
 * @file robot_controller.hpp
 * @brief This file contains the RobotVelocityController class, which manages the
 * control of a robot's motors and implements odometry calculations based on its
 * kinematics model.
 * @version 0.2
 * @date 2024-05-17
 */

#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#ifdef JOINTSTATECONTROLLER_H
#error "joint_state_controller.hpp and robot_controller.hpp cannot be used at the same time!"
#endif

#include <roboost/kinematics/kinematics.hpp>
#include <roboost/motor_control/motor_control_manager.hpp>
#include <roboost/utils/matrices.hpp>

namespace roboost
{
    namespace robot_controller
    {
        template <typename MotorControllerType, typename KinematicsType>
        class RobotVelocityController
        {
        public:
            RobotVelocityController(roboost::motor_control::MotorControllerManager<MotorControllerType>& motor_manager, KinematicsType& kinematics_model)
                : motor_manager_(motor_manager), kinematics_model_(kinematics_model), robot_velocity_(roboost::math::ZeroVector<float>(3)), latest_command_(roboost::math::ZeroVector<float>(3))
            {
            }

            void update()
            {
                // TODO: TYPES???
                roboost::math::Vector<float> desired_wheel_speeds = kinematics_model_.calculate_wheel_velocity(latest_command_);

                int motor_count = motor_manager_.get_motor_count();
                if (desired_wheel_speeds.size() != motor_count)
                {
                    // Serial.println("Not enough motor controllers"); // TODO: Implement logging
                    return;
                }

                for (uint8_t i = 0; i < motor_count; ++i)
                {
                    motor_manager_.set_motor_speed(i, desired_wheel_speeds.at(i));
                }
                motor_manager_.update();

                roboost::math::Vector<float> actual_wheel_speeds(motor_count);
                for (uint8_t i = 0; i < motor_count; ++i)
                {
                    actual_wheel_speeds.at(i) = motor_manager_.get_motor_speed(i);
                }

                robot_velocity_ = kinematics_model_.calculate_robot_velocity(actual_wheel_speeds);
            }

            roboost::math::Vector<float> get_robot_vel() const { return robot_velocity_; }

            roboost::math::Vector<float> get_wheel_vel_setpoints() const { return kinematics_model_.calculate_wheel_velocity(latest_command_); }

            void set_latest_command(const roboost::math::Vector<float>& latest_command) { latest_command_ = latest_command; }

        private:
            roboost::motor_control::MotorControllerManager<MotorControllerType>& motor_manager_;
            KinematicsType& kinematics_model_;

            roboost::math::Vector<float> latest_command_;
            roboost::math::Vector<float> robot_velocity_;
        };
    } // namespace robot_controller
} // namespace roboost

#endif // ROBOT_CONTROLLER_HPP
