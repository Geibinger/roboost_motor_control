/**
 * @file simple_motor_controller.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Implementation of MotorController, which sets the control output
 * directy to the motor driver without feedback loop.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <algorithm>
#include <roboost/motor_control/motor_controllers/simple_motor_controller.hpp>

using namespace roboost::motor_control;

SimpleMotorController::SimpleMotorController(MotorDriver& motor_driver, double max_rotation_speed) : MotorController(motor_driver), max_rotation_speed_(max_rotation_speed) {}

void SimpleMotorController::set_target(double desired_rotation_speed)
{
    desired_rotation_speed = std::clamp(desired_rotation_speed, -max_rotation_speed_, max_rotation_speed_);

    rotation_speed_setpoint_ = desired_rotation_speed / max_rotation_speed_;

    motor_driver_.set_motor_control(rotation_speed_setpoint_);
}

double SimpleMotorController::get_measurement() const { return rotation_speed_setpoint_; }