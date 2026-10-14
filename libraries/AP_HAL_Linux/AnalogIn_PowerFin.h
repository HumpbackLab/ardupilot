#pragma once

#include "AP_HAL_Linux.h"

namespace Linux
{

class AnalogIn_PowerFin : public AP_HAL::AnalogIn
{
public:
    void init() override;
    AP_HAL::AnalogSource *channel(int16_t pin) override;
    float board_voltage() override
    {
        return 5.0f;
    }

private:
    char device_path[128] {};
    float scale_mv = 0;
    float offset = 0;
};

}
