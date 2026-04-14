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
    enum class DistanceMode : uint8_t {
        Short  = 0,  // Up to 1.3m, better ambient immunity
        Medium = 1,  // Up to 3m
        Long   = 2   // Up to 4m, reduced ambient immunity
    };

    RangingSensor(TwoWire& wire);

    /// @brief Initialize sensor
    /// @param sample_period_ms Measurement period in milliseconds
    /// @param mode Distance mode (Short/Medium/Long)
    /// @return true if initialization successful
    bool init(uint32_t sample_period_ms = 100, DistanceMode mode = DistanceMode::Short);

    void startContinuous();
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

    bool dataReady();
    MeasurementData getLastMeasurement() const;
    uint16_t getDistanceMM(float x, float y, float z, float roll, float pitch, float yaw);
    float getDistanceM(float x, float y, float z, float roll, float pitch, float yaw);
    bool isInitialized() const;
    VL53L1X& getSensor();

    /// @brief Initialize SD card for logging
    /// @param cs_pin Chip select pin for SD card
    /// @return true if initialization successful
    bool initSD(uint32_t cs_pin);

    /// @brief Enable or disable measurement logging to SD card
    /// @param enable true to start writing to SD card, false to stop
    void enableLogging(bool enable);

private:
    VL53L1X _sensor;
    TwoWire* _wire;
    bool _initialized;
    uint32_t _sample_period_ms;
    MeasurementData _last_measurement;

    bool _sd_initialized;
    bool _is_logging_enabled;
    uint32_t _sd_cs_pin;

    // Performance improvement variables for SD writing
    File _dataFile;
    uint8_t _flush_counter;
};
