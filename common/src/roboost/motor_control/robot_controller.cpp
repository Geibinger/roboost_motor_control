/**
 * @file robot_controller.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Implementation of the VelocityController class.
 * @version 1.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <roboost/motor_control/robot_controller.hpp>

using namespace roboost::robot_controller;
using namespace roboost::motor_control;
using namespace roboost::kinematics;
using namespace roboost::math;

RobotVelocityController::RobotVelocityController(MotorControllerManager& motor_manager, Kinematics* kinematics_model)
    : motor_manager_(motor_manager), kinematics_model_(kinematics_model), robot_velocity_(Zero(3)), latest_command_(Zero(3))
{
}

void RobotVelocityController::update()
{
    Vector desired_wheel_speeds = kinematics_model_->calculate_wheel_velocity(latest_command_);

    int motor_count = motor_manager_.get_motor_count();
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

    Vector actual_wheel_speeds(motor_count);
    for (int i = 0; i < motor_count; ++i)
    {
        actual_wheel_speeds.at(i) = motor_manager_.get_motor_speed(i);
    }

    robot_velocity_ = kinematics_model_->calculate_robot_velocity(actual_wheel_speeds);
}

Vector RobotVelocityController::get_robot_vel() const
{
    // Return the latest odometry data
    return robot_velocity_;
}

Vector RobotVelocityController::get_wheel_vel_setpoints() const
{
    // Return the latest set wheel velocities
    return kinematics_model_->calculate_wheel_velocity(latest_command_);
}

void RobotVelocityController::set_latest_command(const roboost::math::Vector& latest_command) { latest_command_ = latest_command; }
