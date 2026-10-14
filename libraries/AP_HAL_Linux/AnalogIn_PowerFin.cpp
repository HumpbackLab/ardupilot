#include "AnalogIn_PowerFin.h"

#include <AP_HAL/AP_HAL.h>

#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

namespace Linux
{

namespace
{

bool read_number(int fd, float &value)
{
    char buffer[64] {};
    const ssize_t length = pread(fd, buffer, sizeof(buffer) - 1, 0);
    if (length <= 0 || length == sizeof(buffer) - 1) {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    const float parsed = strtof(buffer, &end);
    if (end == buffer || errno != 0 || !std::isfinite(parsed)) {
        return false;
    }
    while (*end == ' ' || *end == '\n' || *end == '\r' || *end == '\t') {
        end++;
    }
    if (*end != '\0') {
        return false;
    }
    value = parsed;
    return true;
}

bool read_attribute(const char *path, float &value)
{
    const int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        return false;
    }
    const bool ok = read_number(fd, value);
    close(fd);
    return ok;
}

class AnalogSource_PowerFin : public AP_HAL::AnalogSource
{
public:
    AnalogSource_PowerFin(int fd, float scale_mv, float offset) :
        _scale_mv(scale_mv), _offset(offset), _fd(fd)
    {}

    bool set_pin(uint8_t pin) override
    {
        return pin == 0 && sample();
    }

    float read_average() override
    {
        sample();
        return voltage;
    }
    float read_latest() override
    {
        sample();
        return voltage;
    }
    float voltage_average() override
    {
        return voltage;
    }
    float voltage_latest() override
    {
        return read_latest();
    }
    float voltage_average_ratiometric() override
    {
        return voltage_average();
    }

private:
    bool sample()
    {
        float raw;
        if (_fd < 0 || !read_number(_fd, raw)) {
            return false;
        }
        const float measured = (raw + _offset) * _scale_mv * 0.001f;
        if (!std::isfinite(measured) || measured < 0) {
            return false;
        }
        voltage = measured;
        return true;
    }

    const float _scale_mv;
    const float _offset;
    int _fd;
    float voltage = 0;
};

}

void AnalogIn_PowerFin::init()
{
    DIR *devices = opendir("/sys/bus/iio/devices");
    if (devices == nullptr) {
        return;
    }

    while (const dirent *entry = readdir(devices)) {
        if (strncmp(entry->d_name, "iio:device", 10) != 0 || strlen(entry->d_name) > 20) {
            continue;
        }
        char path[sizeof(device_path)];
        snprintf(path, sizeof(path), "/sys/bus/iio/devices/%.20s/name", entry->d_name);
        FILE *name = fopen(path, "r");
        if (name == nullptr) {
            continue;
        }
        char value[64] {};
        const bool matches = fgets(value, sizeof(value), name) != nullptr &&
                             (strcmp(value, "battery-voltage\n") == 0 || strcmp(value, "battery-voltage") == 0);
        fclose(name);
        if (!matches) {
            continue;
        }

        snprintf(path, sizeof(path), "/sys/bus/iio/devices/%.20s/in_voltage0_scale", entry->d_name);
        if (!read_attribute(path, scale_mv) || scale_mv <= 0) {
            break;
        }
        snprintf(path, sizeof(path), "/sys/bus/iio/devices/%.20s/in_voltage0_offset", entry->d_name);
        if (!read_attribute(path, offset) && errno != ENOENT) {
            break;
        }
        snprintf(device_path, sizeof(device_path), "/sys/bus/iio/devices/%.20s/in_voltage0_raw", entry->d_name);
        break;
    }
    closedir(devices);
}

AP_HAL::AnalogSource *AnalogIn_PowerFin::channel(int16_t pin)
{
    if (pin != 0 || device_path[0] == '\0' || scale_mv <= 0) {
        return nullptr;
    }
    const int fd = open(device_path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        return nullptr;
    }
    auto *source = NEW_NOTHROW AnalogSource_PowerFin(fd, scale_mv, offset);
    if (source == nullptr) {
        close(fd);
    }
    return source;
}

}
