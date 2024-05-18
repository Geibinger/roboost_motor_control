/**
 * @file motor_drivers.hpp
 * @brief Defines motor driver interfaces and implementations using the CRTP pattern.
 * @version 0.3
 * @date 2024-05-17
 */

#ifndef MOTOR_DRIVERS_H
#define MOTOR_DRIVERS_H

#include <Arduino.h>
#include <algorithm>
#include <cstdint>
#include <type_traits>

namespace roboost::motor_control
{
    /**
     * @class MotorDriverBase
     * @brief Base class for CRTP motor drivers.
     *
     * @tparam Derived The derived motor driver class.
     * @tparam Bits The number of bits for PWM resolution.
     */
    template <typename Derived>
    class MotorDriverBase
    {
    public:
        /**
         * @brief Set the motor control value.
         *
         * @param control_value The control value for the motor.
         */
        void set_motor_control(int32_t control_value) { static_cast<Derived*>(this)->set_motor_control(control_value); }

        /**
         * @brief Get the motor control value.
         *
         * @return ControlValue The control value.
         */
        int32_t get_motor_control() const { return static_cast<const Derived*>(this)->get_motor_control(); }

    protected:
        volatile int32_t control_value_;
    };

    constexpr uint8_t PWM_RESOLUTION = UINT8_C(10);
    constexpr uint32_t PWM_FREQUENCY = UINT32_C(1 << 11);

    /**
     * @class L298NMotorDriver
     * @brief Implementation of MotorDriver for L298N motor drivers.
     *
     * @tparam Bits The number of bits for PWM resolution.
     */
    class L298NMotorDriver : public MotorDriverBase<L298NMotorDriver>
    {
    public:
        /**
         * @brief Construct a new L298NMotorDriver object
         *
         * @param pin_in1 The pin number of the first input pin.
         * @param pin_in2 The pin number of the second input pin.
         * @param pin_ena The pin number of the enable pin.
         * @param pwm_channel The PWM channel to be used.
         * @param pwm_frequency The PWM frequency.
         */
        L298NMotorDriver(const uint8_t& pin_in1, const uint8_t& pin_in2, const uint8_t& pin_ena, const uint8_t& pwm_channel)
            : pin_in1_(pin_in1), pin_in2_(pin_in2), pin_ena_(pin_ena), pwm_channel_(pwm_channel)
        {
            // Initialize L298N...
            // setting pin modes
            pinMode(pin_in1_, OUTPUT);
            pinMode(pin_in2_, OUTPUT);
            pinMode(pin_ena_, OUTPUT);

#ifdef ESP32
            // configure PWM functionalities
            ledcSetup(pwm_channel_, PWM_FREQUENCY, PWM_RESOLUTION);

            // attach the channel to the GPIO to be controlled
            ledcAttachPin(pin_ena_, pwm_channel_);
#elif defined(TEENSYDUINO)
            analogWriteFrequency(pin_ena_, PWM_FREQUENCY);
            analogWriteResolution(PWM_RESOLUTION);
#endif // ESP32
        }

        /**
         * @brief Set the motor control value.
         *
         * @param control_value The control value to be set.
         * It should be an integer where the sign determines the direction and the magnitude determines the amplitude.
         */
        void set_motor_control(int32_t control_value)
        {
            this->control_value_ = control_value;

            bool direction = control_value >= 0;

#ifdef ESP32
            digitalWrite(pin_in1_, direction ? HIGH : LOW); // TODO: Implement digitalWriteFast for ESP32
            digitalWrite(pin_in2_, direction ? LOW : HIGH);

            ledcWrite(pwm_channel_, std::abs(control_value));
#elif defined(TEENSYDUINO)
            digitalWriteFast(pin_in1_, direction ? HIGH : LOW);
            digitalWriteFast(pin_in2_, direction ? LOW : HIGH);

            analogWrite(pin_ena_, std::abs(control_value));
#endif // ESP32
        }

        /**
         * @brief Get the motor control value.
         *
         * @return ControlValue The control value.
         */
        int32_t get_motor_control() const { return this->control_value_; }

    private:
        const uint8_t pin_in1_;
        const uint8_t pin_in2_;
        const uint8_t pin_ena_;
        const uint8_t pwm_channel_;
    };

} // namespace roboost::motor_control

#endif // MOTOR_DRIVERS_H
