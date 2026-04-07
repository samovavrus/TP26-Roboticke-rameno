#include "RangingSensor.h"

RangingSensor::RangingSensor(TwoWire& wire) 
    : _wire(&wire), _initialized(false), _sample_period_ms(100), _last_measurement{} {}

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
