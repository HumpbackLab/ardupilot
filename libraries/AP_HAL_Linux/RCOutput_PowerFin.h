#pragma once

#include "AP_HAL_Linux.h"

namespace Linux
{

class RCOutput_PowerFin : public AP_HAL::RCOutput
{
public:
    ~RCOutput_PowerFin();

    void init() override;
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void enable_ch(uint8_t ch) override;
    void disable_ch(uint8_t ch) override;
    void write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void read(uint16_t *period_us, uint8_t len) override;
    void cork() override;
    void push() override;
    void set_output_mode(uint32_t mask, enum output_mode mode) override;
    enum output_mode get_output_mode(uint32_t &mask) override;

private:
    static constexpr uint8_t channel_count = 4;

    bool send_frame();

    int fd = -1;
    uint16_t pwm[channel_count] {};
    uint8_t enabled_mask = 0;
    bool corked = false;
    enum output_mode mode = MODE_PWM_NONE;
};

}
