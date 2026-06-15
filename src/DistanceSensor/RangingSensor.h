/**
 * @file RangingSensor.h
 * @brief VL53L1X ranging sensor wrapper with pose tagging and logging.
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "..\VL53L1X\VL53L1X.h"

/// @brief Sensor pose in 3D space (position + orientation)
struct SensorPose {
    float x;        // Position X [m]
    float y;        // Position Y [m]
    float z;        // Position Z [m]
    float roll;     // Rotation around X axis [rad]
    float pitch;    // Rotation around Y axis [rad]
    float yaw;      // Rotation around Z axis [rad]
};

/// @brief Measurement data with pose for map reconstruction
struct MeasurementData {
    uint16_t distance_mm;       // Distance in millimeters
    uint8_t  range_status;      // Sensor status (0 = valid)
    uint16_t signal_rate;       // Signal strength
    uint16_t ambient_rate;      // Ambient light level
    uint32_t timestamp_ms;      // Measurement timestamp
    bool     valid;             // Measurement validity flag
    SensorPose pose;            // Sensor pose at measurement time
};

/// @brief Ranging sensor handler for VL53L1X ToF sensor
/// Designed for future integration with octomap and RAM storage
class RangingSensor {
public:

    static RangingSensor& instance(TwoWire& wire);
    /// @brief Distance measurement mode for the ToF sensor.
    enum class DistanceMode : uint8_t {
        Short  = 0,  // Up to 1.3m, better ambient immunity
        Medium = 1,  // Up to 3m
        Long   = 2   // Up to 4m, reduced ambient immunity
    };

    /**
     * @brief Construct a ranging sensor wrapper on a given I2C bus.
     * @param wire I2C bus instance used by the sensor.
     */
    RangingSensor(TwoWire& wire);

    /// @brief Initialize sensor
    /// @param sample_period_ms Measurement period in milliseconds
    /// @param mode Distance mode (Short/Medium/Long)
    /// @return true if initialization successful
    bool init(uint32_t sample_period_ms = 80, DistanceMode mode = DistanceMode::Short);

    /// @brief Start continuous ranging if initialized.
    void startContinuous();
    /// @brief Stop continuous ranging if initialized.
    void stopContinuous();

    /// @brief Read measurement with current sensor pose
    /// @param x Position X [m]
    /// @param y Position Y [m]
    /// @param z Position Z [m]
    /// @param roll Rotation around X [rad]
    /// @param pitch Rotation around Y [rad]
    /// @param yaw Rotation around Z [rad]
    /// @return MeasurementData with distance and pose
    MeasurementData read(float x, float y, float z, float roll, float pitch, float yaw);

    /// @brief Check if a new measurement is available.
    bool dataReady();
    /// @brief Get the last measurement captured by the sensor.
    MeasurementData getLastMeasurement() const;
    /// @brief Convenience method returning distance in millimeters.
    uint16_t getDistanceMM(float x, float y, float z, float roll, float pitch, float yaw);
    /// @brief Convenience method returning distance in meters.
    float getDistanceM(float x, float y, float z, float roll, float pitch, float yaw);
    /// @brief Check if the sensor was initialized successfully.
    bool isInitialized() const;
    /// @brief Access the underlying VL53L1X driver instance.
    VL53L1X& getSensor();

    /// @brief Initialize SD card for logging
    /// @param cs_pin Chip select pin for SD card
    /// @return true if initialization successful
    bool initSD(uint32_t cs_pin);

    /// @brief Enable or disable measurement logging to SD card
    /// @param enable true to start writing to SD card, false to stop
    /// @param clear true to clear the octomap.txt file before logging
    void enableLogging(bool enable, bool clear = false);

private:
    RangingSensor(TwoWire& wire);

    VL53L1X _sensor;
    TwoWire* _wire;
    bool _initialized;
    uint32_t _sample_period_ms;
    MeasurementData _last_measurement;

    bool _sd_initialized;
    bool _is_logging_enabled;
    uint32_t _sd_cs_pin;

    File _dataFile;
    uint8_t _flush_counter;
};

/**
 * @brief FreeRTOS task entry for distance sensor acquisition.
 * @param pvParameters Unused task parameter.
 */
void TaskSensor(void* pvParameters);

