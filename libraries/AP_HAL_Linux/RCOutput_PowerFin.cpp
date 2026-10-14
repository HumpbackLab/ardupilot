#include "RCOutput_PowerFin.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace Linux
{

namespace
{
struct DShotFrame {
    uint16_t value[4];
};

static_assert(sizeof(DShotFrame) == 8, "PowerFin DShot frame ABI mismatch");

constexpr unsigned long set_rate_ioctl = _IOW('D', 0x00, uint32_t);
constexpr unsigned long set_telemetry_ioctl = _IOW('D', 0x01, uint32_t);
constexpr unsigned long send_frame_ioctl = _IOW('D', 0x03, DShotFrame);
}

RCOutput_PowerFin::~RCOutput_PowerFin()
{
    if (fd >= 0) {
        DShotFrame stop {};
        ioctl(fd, send_frame_ioctl, &stop);
        close(fd);
    }
}

void RCOutput_PowerFin::init()
{
    fd = open("/dev/rk-flexbus-dshot", O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        AP_HAL::panic("PowerFin: DShot device unavailable");
    }

    const uint32_t rate = 300000;
    if (ioctl(fd, set_rate_ioctl, &rate) < 0) {
        AP_HAL::panic("PowerFin: cannot set DShot rate");
    }

    const uint32_t telemetry = 0;
    if (ioctl(fd, set_telemetry_ioctl, &telemetry) < 0) {
        AP_HAL::panic("PowerFin: cannot configure DShot telemetry");
    }

    if (!send_frame()) {
        AP_HAL::panic("PowerFin: cannot stop DShot outputs");
    }
}

void RCOutput_PowerFin::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    (void)chmask;
    (void)freq_hz;
}

uint16_t RCOutput_PowerFin::get_freq(uint8_t ch)
{
    (void)ch;
    return 0;
}

void RCOutput_PowerFin::enable_ch(uint8_t ch)
{
    if (ch < channel_count) {
        enabled_mask |= 1U << ch;
    }
}

void RCOutput_PowerFin::disable_ch(uint8_t ch)
{
    if (ch < channel_count) {
        enabled_mask &= ~(1U << ch);
        pwm[ch] = 0;
        if (!corked && !send_frame()) {
            AP_HAL::panic("PowerFin: DShot stop failed");
        }
    }
}

void RCOutput_PowerFin::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= channel_count) {
        return;
    }
    pwm[ch] = period_us;
    if (!corked && !send_frame()) {
        AP_HAL::panic("PowerFin: DShot write failed");
    }
}

uint16_t RCOutput_PowerFin::read(uint8_t ch)
{
    return ch < channel_count ? pwm[ch] : 0;
}

void RCOutput_PowerFin::read(uint16_t *period_us, uint8_t len)
{
    for (uint8_t ch = 0; ch < len; ch++) {
        period_us[ch] = read(ch);
    }
}

void RCOutput_PowerFin::cork()
{
    corked = true;
}

void RCOutput_PowerFin::push()
{
    if (corked) {
        corked = false;
        if (!send_frame()) {
            AP_HAL::panic("PowerFin: DShot write failed");
        }
    }
}

void RCOutput_PowerFin::set_output_mode(uint32_t mask, enum output_mode new_mode)
{
    if ((mask & 0x0f) == 0) {
        return;
    }
    if (!is_dshot_protocol(new_mode)) {
        mode = MODE_PWM_NONE;
        memset(pwm, 0, sizeof(pwm));
        if (!send_frame()) {
            AP_HAL::panic("PowerFin: DShot stop failed");
        }
        return;
    }

    uint32_t rate;
    switch (new_mode) {
    case MODE_PWM_DSHOT150:
        rate = 150000;
        break;
    case MODE_PWM_DSHOT300:
        rate = 300000;
        break;
    case MODE_PWM_DSHOT600:
        rate = 600000;
        break;
    case MODE_PWM_DSHOT1200:
        rate = 1200000;
        break;
    default:
        return;
    }
    if (ioctl(fd, set_rate_ioctl, &rate) < 0) {
        AP_HAL::panic("PowerFin: cannot set DShot rate");
    }
    mode = new_mode;
}

AP_HAL::RCOutput::output_mode RCOutput_PowerFin::get_output_mode(uint32_t &mask)
{
    mask = 0x0f;
    return mode;
}

bool RCOutput_PowerFin::send_frame()
{
    if (fd < 0) {
        return false;
    }
    DShotFrame frame {};
    if (is_dshot_protocol(mode)) {
        for (uint8_t ch = 0; ch < channel_count; ch++) {
            if ((enabled_mask & (1U << ch)) == 0 || pwm[ch] <= 1000) {
                continue;
            }
            const uint16_t value = 2U * (constrain_int16(pwm[ch], 1000, 2000) - 1000);
            frame.value[ch] = value + 47;
        }
    }
    if (ioctl(fd, send_frame_ioctl, &frame) == 0) {
        return true;
    }
    DShotFrame stop {};
    ioctl(fd, send_frame_ioctl, &stop);
    return false;
}

}
