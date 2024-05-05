/**
 * @file robot_controller.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief This file contains the RobotVelocityController class, which manages the
 * control of a robot's motors and implements odometry calculations based on its
 * kinematics model.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef VELOCITYCONTROLLER_H
#define VELOCITYCONTROLLER_H

#ifdef JOINTSTATECONTROLLER_H
#error "joint_state_controller.hpp and robot_controller.hpp cannot be used at the same time!"
#endif

#include <roboost/kinematics/kinematics.hpp>
#include <roboost/motor_control/motor_control_manager.hpp>

namespace roboost
{
    namespace robot_controller
    {
        // TODO: Implement JointStateController class
        /**
         * @brief The VelocityController class manages the control of a robot's motors
         * and implements odometry calculations based on its kinematics model.
         */
        class RobotVelocityController
        {
        public:
            /**
             * @brief Construct a new Robot Controller object.
             *
             * @param motor_manager The motor control manager responsible for motor
             *                      control.
             * @param kinematics_model The kinematics model used for odometry
             * calculations.
             */
            RobotVelocityController(roboost::motor_control::MotorControllerManager& motor_manager, roboost::kinematics::Kinematics* kinematics_model);

            /**
             * @brief Update the robot's control loop. This method should be called
             *        periodically to control the robot's motors and update odometry.
             */
            void update();

            /**
             * @brief Get the current velocity estimation estimation.
             *
             * @return roboost::math::Vector The current robot velocity estimation.
             */
            roboost::math::Vector get_robot_vel() const;

            /**
             * @brief Get the current set wheel velocities.
             *
             * @return roboost::math::Vector The current set wheel velocities based on latest
             * command.
             */
            roboost::math::Vector get_wheel_vel_setpoints() const;

            /**
             * @brief Set the latest command for the robot's motion control.
             *
             * @param latest_command A vector containing the latest command for the
             * robot's motion control, currently only representing linear velocities
             * (vx, vy, vz).
             */
            void set_latest_command(const roboost::math::Vector& latest_command);

        private:
            roboost::motor_control::MotorControllerManager& motor_manager_;
            roboost::kinematics::Kinematics* kinematics_model_;

            roboost::math::Vector latest_command_;
            roboost::math::Vector robot_velocity_;
        };
    } // namespace robot_controller
} // namespace roboost

#endif // VELOCITYCONTROLLER_H
