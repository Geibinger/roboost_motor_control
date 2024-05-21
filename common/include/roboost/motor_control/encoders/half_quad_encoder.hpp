/**
 * @file encoder.hpp
 * @brief This file contains the implementation class for reading encoder values.
 * @version 0.4
 * @date 2024-05-17
 */

#ifndef ENCODER_HPP
#define ENCODER_HPP

#ifdef ARDUINO
#include <Arduino.h>
#endif

#ifdef ESP32
#include <ESP32Encoder.h>
#endif

#include <roboost/motor_control/encoders/encoder.hpp>
#include <roboost/utils/callback_scheduler.hpp>
#include <roboost/utils/logging.hpp>
#include <roboost/utils/time_macros.hpp>

namespace roboost::motor_control
{
#ifdef ESP32

    /**
     * @brief Encoder class for quadrature encoders.
     */
    class HalfQuadEncoder : public EncoderBase
    {
    public:
        /**
         * @brief Construct a new HalfQuadEncoder object
         *
         * @param pin_A The pin for the A channel.
         * @param pin_B The pin for the B channel.
         * @param resolution The resolution of the encoder.
         * @param reverse Whether the encoder is reversed.
         */
        HalfQuadEncoder(const uint8_t& pin_A, const uint8_t& pin_B, const uint16_t& resolution, const bool reverse = false)
            : resolution_(resolution), step_increment_(2.0 * M_PI / resolution), reverse_(reverse), prev_count_(0)
        {
            encoder_.attachSingleEdge(pin_A, pin_B);
            last_position_change_time_ = micros();
        }

        /**
         * @brief Implementation of get_velocity() for the derived class.
         *
         * @return int64_t The velocity in ticks/s
         */
        int64_t get_velocity() const override { return velocity_; }

        /**
         * @brief Implementation of get_position() for the derived class.
         *
         * @return int64_t The position in ticks
         */
        int64_t get_position() const override { return position_; }

        /**
         * @brief Get the step increment in radians per tick.
         *
         * @return float The step increment.
         */
        float get_step_increment() const override { return step_increment_; }

        /**
         * @brief Implementation of update() for the derived class.
         *
         */
        void update() override
        {
            // uint32_t dt = timing::CallbackScheduler::get_instance().get_delta_time(); // TODO: Use frequency instead of delta time
            // if (dt == 0)
            //     return; // Prevent division by zero

            int64_t count = encoder_.getCount();

            // TODO: This is a hack, please fix
            // check if position has changed
            int64_t position_change = (count - prev_count_) * (reverse_ ? -1 : 1);

            // if position has changed, calculate velocity based on time between last position change
            if (position_change != 0)
            {
                uint32_t time = micros();
                uint32_t time_diff = time - last_position_change_time_;
                velocity_ = (position_change * 1000000) / time_diff; // ticks per second
                last_position_change_time_ = time;
            }
            else if (micros() - last_position_change_time_ > 50000)
                // TODO: Change this
                // If position has not changed for a while, set velocity to 0
                velocity_ = 0;

            // velocity_ = (position_change * 1000000) / dt; // ticks per second

            position_ = count;
            prev_count_ = count;
        }

    private:
        const uint16_t resolution_;  // number of steps per revolution
        const float step_increment_; // in radians per step
        const bool reverse_;
        volatile int64_t prev_count_;
        volatile int64_t position_ = 0; // in ticks
        volatile int64_t velocity_ = 0; // in ticks per second
        ESP32Encoder encoder_;

        // TODO: Remove this
        uint32_t last_position_change_time_;
    };

#endif // ESP32

} // namespace roboost::motor_control

#endif // ENCODER_HPP
