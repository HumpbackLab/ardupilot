#pragma once

#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_SC7U22IIO : public AP_InertialSensor_Backend
{
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu, enum Rotation rotation);
    ~AP_InertialSensor_SC7U22IIO() override;

    void start() override;
    void accumulate() override;
    bool update() override;

private:
    AP_InertialSensor_SC7U22IIO(AP_InertialSensor &imu, enum Rotation rotation);

    bool configure();
    void disable();
    bool read_scans();

    enum Rotation sensor_rotation;
    char sysfs_path[128] {};
    int fd = -1;
    bool buffer_enabled = false;
    uint64_t last_timestamp_us = 0;
};
