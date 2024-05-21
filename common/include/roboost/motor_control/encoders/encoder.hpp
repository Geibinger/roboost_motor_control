/**
 * @file encoder_base.hpp
 * @brief This file contains the abstract base class for reading encoder values.
 * @version 0.4
 * @date 2024-05-17
 */

#ifndef ENCODER_BASE_HPP
#define ENCODER_BASE_HPP

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include <cstdint>

namespace roboost::motor_control
{
    /**
     * @brief Abstract base class for reading encoder values.
     */
    class EncoderBase
    {
    public:
        virtual ~EncoderBase() = default;

        /**
         * @brief Get the velocity of the encoder.
         *
         * @return int64_t The velocity in ticks/s.
         */
        virtual int64_t get_velocity() const = 0;

        /**
         * @brief Get the position of the encoder.
         *
         * @return int64_t The position in ticks.
         */
        virtual int64_t get_position() const = 0;

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
        virtual void update() = 0;

        /**
         * @brief Get the step increment in radians per tick.
         *
         * @return float The step increment.
         */
        virtual float get_step_increment() const = 0;

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

} // namespace roboost::motor_control

#endif // ENCODER_BASE_HPP
