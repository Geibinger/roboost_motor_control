/**
 * @file encoder.hpp
 * @brief This file contains the abstract base class for reading encoder values.
 * @version 0.4
 * @date 2024-05-17
 */

#ifndef ENCODER_H
#define ENCODER_H

#ifdef ARDUINO
#include <Arduino.h>
#endif

#ifdef ESP32
#include <ESP32Encoder.h>
#endif

// TODO: Add Teensyduino support

#include <roboost/utils/callback_scheduler.hpp>
#include <roboost/utils/logging.hpp>
#include <roboost/utils/time_macros.hpp>

namespace roboost::motor_control
{

    /**
     * @brief Encoder base class using CRTP.
     *
     * @tparam Derived The derived class implementing the Encoder interface.
     */
    template <typename Derived>
    class Encoder
    {
    public:
        /**
         * @brief Get the velocity of the encoder.
         *
         * @return int64_t The velocity in ticks/s.
         */
        int64_t get_velocity() const { return static_cast<const Derived*>(this)->get_velocity(); }

        /**
         * @brief Get the position of the encoder.
         *
         * @return int64_t The position in ticks.
         */
        int64_t get_position() const { return static_cast<const Derived*>(this)->get_position(); }

        /**
         * @brief Get the velocity of the encoder in radians per second.
         *
         * @return float The velocity in radians per second.
         */
        float get_velocity_radians_per_second() const { return ticks_to_radians_per_second(get_velocity()); }

        /**
         * @brief Get the position of the encoder in radians.
         *
         * @return float The position in radians.
         */
        float get_position_radians() const { return ticks_to_radians(get_position()); }

        /**
         * @brief Update the encoder values.
         *
         * @note This function should be called regularly to update the encoder values.
         */
        void update() { static_cast<Derived*>(this)->update(); }

        /**
         * @brief Get the step increment in radians per tick.
         *
         * @return float The step increment.
         */
        inline float get_step_increment() const { return static_cast<const Derived*>(this)->get_step_increment(); }

        /**
         * @brief Convert ticks to radians.
         *
         * @param ticks The number of ticks.
         * @return float The equivalent radians.
         */
        inline float ticks_to_radians(const int64_t& ticks) const { return ticks * get_step_increment(); }

        /**
         * @brief Convert ticks/s to rad/s.
         *
         * @param ticks_per_second The velocity in ticks per second.
         * @return float The equivalent radians per second.
         */
        inline float ticks_to_radians_per_second(const int64_t& ticks_per_second) const { return ticks_per_second * get_step_increment(); }
    };

#ifdef ESP32

    /**
     * @brief Encoder class for quadrature encoders.
     *
     */
    class HalfQuadEncoder : public Encoder<HalfQuadEncoder>
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
            : resolution_(resolution), step_increment_(2.0 * M_PI / resolution), reverse_(reverse), prev_count_(0), timing_service_(roboost::timing::CallbackScheduler::get_instance())
        {
            encoder_.attachSingleEdge(pin_A, pin_B);
        }

        /**
         * @brief Implementation of get_velocity() for the derived class.
         *
         * @return int64_t The velocity in ticks/s
         */
        int64_t get_velocity() const { return velocity_; }

        /**
         * @brief Implementation of get_position() for the derived class.
         *
         * @return int64_t The position in ticks
         */
        int64_t get_position() const { return position_; }

        /**
         * @brief Get the step increment in radians per tick.
         *
         * @return float The step increment.
         */
        float get_step_increment() const { return step_increment_; }

        /**
         * @brief Implementation of update() for the derived class.
         *
         */
        void update()
        {
            uint32_t dt = timing_service_.get_delta_time();
            if (dt == 0)
                return; // Prevent division by zero

            int64_t count = encoder_.getCount();
            int64_t position_change = (count - prev_count_) * (reverse_ ? -1 : 1);

            position_ = count;
            velocity_ = (position_change * 1000000) / dt; // ticks per second

            prev_count_ = count;
        }

    private:
        const uint16_t resolution_;  // number of steps per revolution
        const float step_increment_; // in radians per step
        const bool reverse_;
        volatile int64_t prev_count_;
        volatile int64_t position_ = 0;             // in ticks
        volatile int64_t velocity_ = 0;             // in ticks per second
        ESP32Encoder encoder_;                      // TODO: use pointer instead of object
        timing::CallbackScheduler& timing_service_; // TODO: use pointer instead of object
    };

#endif // ESP32

} // namespace roboost::motor_control

#endif // ENCODER_H
