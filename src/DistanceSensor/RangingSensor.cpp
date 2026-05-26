/**
 * @file RangingSensor.cpp
 * @brief Implementation of the VL53L1X ranging sensor wrapper.
 */
#include "RangingSensor.h"
#include <STM32FreeRTOS.h>

RangingSensor::RangingSensor(TwoWire& wire) 
    : _wire(&wire), _initialized(false), _sample_period_ms(80), _last_measurement{},
      _sd_initialized(false), _is_logging_enabled(false), _sd_cs_pin(0), _flush_counter(0) {}

bool RangingSensor::init(uint32_t sample_period_ms, DistanceMode mode) {
    _sample_period_ms = sample_period_ms;
    
    _sensor.setBus(_wire);
    _sensor.setTimeout(sample_period_ms * 2);
    
    if (!_sensor.init()) {
        return false;
    }

    // Set timing budget (80% of sample period for measurement)
    uint32_t timing_budget_us = (uint32_t)(0.8f * sample_period_ms * 1000);
    _sensor.setMeasurementTimingBudget(timing_budget_us);
    
    switch (mode) {
        case DistanceMode::Short:
            _sensor.setDistanceMode(VL53L1X::Short);
            break;
        case DistanceMode::Medium:
            _sensor.setDistanceMode(VL53L1X::Medium);
            break;
        case DistanceMode::Long:
            _sensor.setDistanceMode(VL53L1X::Long);
            break;
    }

    _initialized = true;
    return true;
}

void RangingSensor::startContinuous() {
    if (_initialized) {
        _sensor.startContinuous(_sample_period_ms);
    }
}

void RangingSensor::stopContinuous() {
    if (_initialized) {
        _sensor.stopContinuous();
    }
}

MeasurementData RangingSensor::read(float x, float y, float z, float roll, float pitch, float yaw) {
    MeasurementData data = {};
    
    if (!_initialized) {
        data.valid = false;
        return data;
    }

    // Store pose
    data.pose.x = x;
    data.pose.y = y;
    data.pose.z = z;
    data.pose.roll = roll;
    data.pose.pitch = pitch;
    data.pose.yaw = yaw;

    // Read sensor
    data.distance_mm = _sensor.read();
    data.range_status = _sensor.ranging_data.range_status;
    data.signal_rate = _sensor.ranging_data.peak_signal_count_rate_MCPS;
    data.ambient_rate = _sensor.ranging_data.ambient_count_rate_MCPS;
    data.timestamp_ms = millis();
    data.valid = (data.range_status == 0) && !_sensor.timeoutOccurred();


    if (_is_logging_enabled && _sd_initialized && data.valid) {
        if (_dataFile) {
            _dataFile.print(data.timestamp_ms);
            _dataFile.print(",");
            _dataFile.print(data.pose.x, 6);
            _dataFile.print(",");
            _dataFile.print(data.pose.y, 6);
            _dataFile.print(",");
            _dataFile.print(data.pose.z, 6);
            _dataFile.print(",");
            _dataFile.print(data.pose.roll, 6);
            _dataFile.print(",");
            _dataFile.print(data.pose.pitch, 6);
            _dataFile.print(",");
            _dataFile.print(data.pose.yaw, 6);
            _dataFile.print(",");
            _dataFile.println(data.distance_mm);

            _flush_counter++;
            if (_flush_counter >= 20) { 
                _dataFile.flush();
                _flush_counter = 0;
            }
        }
    }

    _last_measurement = data;
    return data;
}

bool RangingSensor::dataReady() {
    return _initialized && _sensor.dataReady();
}

MeasurementData RangingSensor::getLastMeasurement() const {
    return _last_measurement;
}

uint16_t RangingSensor::getDistanceMM(float x, float y, float z, float roll, float pitch, float yaw) {
    MeasurementData data = read(x, y, z, roll, pitch, yaw);
    return data.valid ? data.distance_mm : 0;
}

float RangingSensor::getDistanceM(float x, float y, float z, float roll, float pitch, float yaw) {
    return getDistanceMM(x, y, z, roll, pitch, yaw) / 1000.0f;
}

bool RangingSensor::isInitialized() const {
    return _initialized;
}

VL53L1X& RangingSensor::getSensor() {
    return _sensor;
}

bool RangingSensor::initSD(uint32_t cs_pin) {
    _sd_cs_pin = cs_pin;

    SPI.setMISO(PE5);
    SPI.setMOSI(PE6);
    SPI.setSCLK(PE2);
    
    if (SD.begin(cs_pin)) {
        _sd_initialized = true;
        return true;
    }
    
    _sd_initialized = false;
    return false;
}

void RangingSensor::enableLogging(bool enable, bool clear) {
    if (_sd_initialized) {
        if (enable) {
            if (!_is_logging_enabled) {
                if (clear) {
                    SD.remove("pointcloud.ply");
                }
                _dataFile = SD.open("pointcloud.ply", FILE_WRITE);
                _flush_counter = 0;
                _is_logging_enabled = true;
            } else if (clear) {
                if (_dataFile) {
                    _dataFile.flush();
                    _dataFile.close();
                }
                SD.remove("pointcloud.ply");
                _dataFile = SD.open("pointcloud.ply", FILE_WRITE);
                _flush_counter = 0;
            }
        } else {
            if (_is_logging_enabled) {
                if (_dataFile) {
                    _dataFile.flush();
                    _dataFile.close();
                }
                _is_logging_enabled = false;
            }
        }
    }
}

/**
 * @brief FreeRTOS task for periodic VL53L1X ranging and optional logging.
 * @param pvParameters Unused task parameter.
 */
void TaskSensor(void* pvParameters) {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    
    const uint32_t Ts_ms = 80; 
    // I2C bus: SDA=PB4 (D5), SCL=PA8 (D7)
    TwoWire Wire3(PB4, PA8);
    Wire3.begin();
    Wire3.setClock(400000);
    RangingSensor rangingSensor(Wire3);
    if (!rangingSensor.init(Ts_ms, RangingSensor::DistanceMode::Short)) {
        Serial.println("RangingSensor init failed!");
        vTaskDelete(NULL);
        return;
    }

    if (rangingSensor.initSD(PE4)) {
        Serial.println("SD Card initialized for RangingSensor!");
        rangingSensor.enableLogging(true, true);
    } else {
        Serial.println("SD Card init failed! Logging disabled.");
    }

    rangingSensor.startContinuous();
    Serial.println("RangingSensor initialized");

    while (1) {
        MeasurementData measurement = rangingSensor.read(x, y, z, roll, pitch, yaw);
        
        if (measurement.valid) {
            Serial.print("Distance: ");
            Serial.print(measurement.distance_mm);
            Serial.print(" mm | Pose: (");
            Serial.print(measurement.pose.x, 3); Serial.print(", ");
            Serial.print(measurement.pose.y, 3); Serial.print(", ");
            Serial.print(measurement.pose.z, 3); Serial.println(")");
        }

        vTaskDelay(pdMS_TO_TICKS(Ts_ms));
    }
}

