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
        /**
         * @brief The RobotVelocityController class manages the control of a robot's motors
         * and implements odometry calculations based on its kinematics model.
         */
        template <typename MotorControllerType, typename KinematicsType>
        class RobotVelocityController
        {
        public:
            /**
             * @brief Construct a new Robot Controller object.
             *
             * @param motor_manager The motor control manager responsible for motor control.
             * @param kinematics_model The kinematics model used for odometry calculations.
             */
            RobotVelocityController(roboost::motor_control::MotorControllerManager<MotorControllerType>& motor_manager, KinematicsType& kinematics_model)
                : motor_manager_(motor_manager), kinematics_model_(kinematics_model), robot_velocity_(roboost::math::ZeroVector<double>(3)), latest_command_(roboost::math::ZeroVector<double>(3))
            {
            }

            /**
             * @brief Update the robot's control loop. This method should be called
             *        periodically to control the robot's motors and update odometry.
             */
            void update()
            {
                roboost::math::Vector<double> desired_wheel_speeds = kinematics_model_.calculate_wheel_velocity(latest_command_);

                int motor_count = motor_manager_.get_motor_count(); // TODO: Make this compile time constant
                if (desired_wheel_speeds.size() != motor_count)
                {
                    // Serial.println("Not enough motor controllers"); // TODO: Implement logging
                    return;
                }

                for (int i = 0; i < motor_count; ++i)
                {
                    motor_manager_.set_motor_speed(i, desired_wheel_speeds.at(i));
                }
                motor_manager_.update();

                roboost::math::Vector<double> actual_wheel_speeds(motor_count);
                for (int i = 0; i < motor_count; ++i)
                {
                    actual_wheel_speeds.at(i) = motor_manager_.get_motor_speed(i);
                }

                robot_velocity_ = kinematics_model_.calculate_robot_velocity(actual_wheel_speeds);
            }

            /**
             * @brief Get the current velocity estimation.
             *
             * @return roboost::math::Vector The current robot velocity estimation.
             */
            roboost::math::Vector<double> get_robot_vel() const
            {
                // Return the latest odometry data
                return robot_velocity_;
            }

            /**
             * @brief Get the current set wheel velocities.
             *
             * @return roboost::math::Vector The current set wheel velocities based on latest command.
             */
            roboost::math::Vector<double> get_wheel_vel_setpoints() const
            {
                // Return the latest set wheel velocities
                return kinematics_model_.calculate_wheel_velocity(latest_command_);
            }

            /**
             * @brief Set the latest command for the robot's motion control.
             *
             * @param latest_command A vector containing the latest command for the
             * robot's motion control, currently only representing linear velocities
             * (vx, vy, vz).
             */
            void set_latest_command(const roboost::math::Vector<double>& latest_command) { latest_command_ = latest_command; }

        private:
            roboost::motor_control::MotorControllerManager<MotorControllerType>& motor_manager_;
            KinematicsType& kinematics_model_;

            roboost::math::Vector<double> latest_command_;
            roboost::math::Vector<double> robot_velocity_;
        };
    } // namespace robot_controller
} // namespace roboost

#endif // ROBOT_CONTROLLER_HPP
