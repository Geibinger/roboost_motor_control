/**
 * @file motor_controller.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief This file contains the implementation of a PID motor controller.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <roboost/motor_control/motor_controllers/position_motor_controller.hpp>
#include <roboost/utils/comparisons.hpp>

#include <algorithm>
#include <cmath>

using namespace roboost::motor_control;
using namespace roboost::controllers;
using namespace roboost::filters;

PositionController::PositionController(MotorDriver& motor_driver, Encoder& encoder, PIDController& pid, Filter& input_filter, Filter& output_filter, RateLimitingFilter& rate_limiting_filter,
                                       double deadband_threshold, double minimum_output)
    : MotorController(motor_driver), encoder_(encoder), pid_(pid), input_filter_(input_filter), output_filter_(output_filter), rate_limiting_filter_(rate_limiting_filter),
      deadband_threshold_(deadband_threshold), minimum_output_(minimum_output), current_setpoint_(0.0)
{
}

void PositionController::set_target(double desired_angle)
{
    encoder_.update();
    double input = encoder_.get_angle();
    input = input_filter_.update(input);

    // Update using the new Rate Limited Filter
    current_setpoint_ = rate_limiting_filter_.update(desired_angle, input);

    double output = pid_.update(current_setpoint_, input);
    output = output_filter_.update(output);

    // Adjust output for deadband and minimum output
    if (fabs(output) < deadband_threshold_)
    {
        output = 0.0;
    }
    else if (fabs(output) < minimum_output_)
    {
        output = minimum_output_ * (output / fabs(output));
    }

    motor_driver_.set_motor_control(output);
}
