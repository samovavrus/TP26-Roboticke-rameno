#pragma once

/**
 * @brief Simple analog joystick interface for Arduino-like environments.
 *
 * This class reads two analog axes (X, Y) from a thumbstick or similar device,
 * normalizes them to the range [-1.0, 1.0], and applies a configurable deadzone.
 *
 * ## Hardware assumptions
 * - Joystick X and Y axes are connected to analog input pins.
 * - Analog range is 0..1023 (typical Arduino `analogRead` behavior).
 * - Neutral (center) position is approximately 512.
 *
 * ## Features
 * - Deadzone filtering to eliminate small noise near center.
 * - Access to both normalized values and raw analog readings.
 * - Symmetric scaling so that -1.0 corresponds to minimum analog reading and
 *   +1.0 to maximum analog reading.
 */
class Joystick {
public:
    /**
     * @brief Construct a Joystick object.
     *
     * @param pinX Analog input pin for X-axis.
     * @param pinY Analog input pin for Y-axis.
     * @param deadzone Fractional deadzone range around center, default = 0.03 (3%).
     *
     * Example:
     * @code
     * Joystick js(A0, A1, 0.05f); // 5% deadzone
     * @endcode
     */
    Joystick(uint8_t pinX, uint8_t pinY, float deadzone = 0.03f)
        : pinX(pinX), pinY(pinY), deadzone(deadzone) {}

    /**
     * @brief Initialize the joystick pins as analog inputs.
     *
     * Must be called before `read()`.
     */
    void begin() {
        pinMode(pinX, INPUT);
        pinMode(pinY, INPUT);
    }

    /**
     * @brief Read joystick positions from hardware, normalize, and apply deadzone.
     *
     * Steps:
     * 1. Read raw analog values from both pins.
     * 2. Convert them to normalized [-1.0, 1.0] range.
     * 3. Apply deadzone by setting small values near 0 to exactly 0.
     *
     * This function should be called frequently in the main loop.
     */
    void read() {
        rawX = analogRead(pinX);
        rawY = analogRead(pinY);

        // Normalize to [-1, 1]
        x = mapToUnit(rawX);
        y = mapToUnit(rawY);

        // Apply deadzone threshold
        if (fabs(x) < deadzone) x = 0.0f;
        if (fabs(y) < deadzone) y = 0.0f;
    }

    /**
     * @brief Get last-read X-axis value (normalized).
     * @return X position in range [-1.0, 1.0] after deadzone.
     */
    float getX() const { return x; }

    /**
     * @brief Get last-read Y-axis value (normalized).
     * @return Y position in range [-1.0, 1.0] after deadzone.
     */
    float getY() const { return y; }

private:
    uint8_t pinX;      ///< Analog pin for X-axis
    uint8_t pinY;      ///< Analog pin for Y-axis
    int rawX;          ///< Last raw analog value for X-axis
    int rawY;          ///< Last raw analog value for Y-axis
    float x;           ///< Last normalized X value [-1, 1]
    float y;           ///< Last normalized Y value [-1, 1]
    float deadzone;    ///< Deadzone size as a fraction of full scale

    /**
     * @brief Convert raw analog value (0..1023) to normalized [-1, 1] range.
     *
     * @param value Raw analog reading.
     * @return Normalized value clamped to [-1.0, 1.0].
     *
     * Formula:
     * \f[
     * norm = \frac{\text{value} - 512}{512}
     * \f]
     *
     * Values beyond ±1.0 due to hardware tolerance are clamped.
     */
    float mapToUnit(int value) {
        float norm = (value - 512) / 512.0f;
        if (norm > 1.0f) norm = 1.0f;
        if (norm < -1.0f) norm = -1.0f;
        return norm;
    }
};