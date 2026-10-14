#include "AP_InertialSensor_SC7U22IIO.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POWERFIN

#include <AP_HAL/AP_HAL.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>

namespace
{

constexpr uint16_t sample_rate_hz = 1600;
constexpr size_t scan_size = 24;
constexpr struct {
    const char *name;
    const char *type;
    uint8_t index;
} scan_elements[] = {
    {"in_accel_x", "le:s16/16>>0", 0},
    {"in_accel_y", "le:s16/16>>0", 1},
    {"in_accel_z", "le:s16/16>>0", 2},
    {"in_anglvel_x", "le:s16/16>>0", 3},
    {"in_anglvel_y", "le:s16/16>>0", 4},
    {"in_anglvel_z", "le:s16/16>>0", 5},
    {"in_timestamp", "le:s64/64>>0", 6},
};

bool read_text(const char *path, char *value, size_t length)
{
    const int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        return false;
    }
    const ssize_t count = read(fd, value, length - 1);
    close(fd);
    if (count <= 0 || static_cast<size_t>(count) == length - 1) {
        return false;
    }
    value[count] = '\0';
    const size_t text_length = strlen(value);
    if (text_length > 0 && value[text_length - 1] == '\n') {
        value[text_length - 1] = '\0';
    }
    return true;
}

bool write_text(const char *path, const char *value)
{
    const int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        return false;
    }
    const size_t length = strlen(value);
    const bool ok = write(fd, value, length) == static_cast<ssize_t>(length);
    close(fd);
    return ok;
}

int16_t read_le16(const uint8_t *data)
{
    return static_cast<int16_t>(uint16_t(data[0]) | (uint16_t(data[1]) << 8));
}

int64_t read_le64(const uint8_t *data)
{
    uint64_t value = 0;
    for (uint8_t i = 0; i < 8; i++) {
        value |= uint64_t(data[i]) << (i * 8);
    }
    return static_cast<int64_t>(value);
}

}

AP_InertialSensor_SC7U22IIO::AP_InertialSensor_SC7U22IIO(AP_InertialSensor &imu, enum Rotation rotation) :
    AP_InertialSensor_Backend(imu), sensor_rotation(rotation)
{
}

AP_InertialSensor_SC7U22IIO::~AP_InertialSensor_SC7U22IIO()
{
    disable();
    if (fd >= 0) {
        close(fd);
    }
}

AP_InertialSensor_Backend *AP_InertialSensor_SC7U22IIO::probe(AP_InertialSensor &imu, enum Rotation rotation)
{
    auto *sensor = NEW_NOTHROW AP_InertialSensor_SC7U22IIO(imu, rotation);
    if (sensor != nullptr && !sensor->configure()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_InertialSensor_SC7U22IIO::configure()
{
    DIR *devices = opendir("/sys/bus/iio/devices");
    if (devices == nullptr) {
        return false;
    }
    char path[192];
    char value[64];
    while (const dirent *entry = readdir(devices)) {
        if (strncmp(entry->d_name, "iio:device", 10) != 0 || strlen(entry->d_name) > 20) {
            continue;
        }
        snprintf(path, sizeof(path), "/sys/bus/iio/devices/%.20s/name", entry->d_name);
        if (read_text(path, value, sizeof(value)) && strcmp(value, "sc7u22") == 0) {
            snprintf(sysfs_path, sizeof(sysfs_path), "/sys/bus/iio/devices/%.20s", entry->d_name);
            break;
        }
    }
    closedir(devices);
    if (sysfs_path[0] == '\0') {
        return false;
    }

    snprintf(path, sizeof(path), "%s/buffer/enable", sysfs_path);
    write_text(path, "0");
    snprintf(path, sizeof(path), "%s/current_timestamp_clock", sysfs_path);
    if (!write_text(path, "monotonic") ||
        !read_text(path, value, sizeof(value)) || strcmp(value, "monotonic") != 0) {
        return false;
    }

    for (const auto &element : scan_elements) {
        snprintf(path, sizeof(path), "%s/scan_elements/%s_type", sysfs_path, element.name);
        if (!read_text(path, value, sizeof(value)) || strcmp(value, element.type) != 0) {
            return false;
        }
        snprintf(path, sizeof(path), "%s/scan_elements/%s_index", sysfs_path, element.name);
        if (!read_text(path, value, sizeof(value)) || atoi(value) != element.index) {
            return false;
        }
        snprintf(path, sizeof(path), "%s/scan_elements/%s_en", sysfs_path, element.name);
        if (!write_text(path, "1")) {
            return false;
        }
    }

    snprintf(path, sizeof(path), "%s/buffer/length", sysfs_path);
    if (!write_text(path, "256")) {
        return false;
    }
    snprintf(path, sizeof(path), "%s/buffer/watermark", sysfs_path);
    write_text(path, "2");

    const char *device_name = strrchr(sysfs_path, '/');
    snprintf(path, sizeof(path), "/dev/%s", device_name + 1);
    fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        return false;
    }
    snprintf(path, sizeof(path), "%s/buffer/enable", sysfs_path);
    if (!write_text(path, "1")) {
        return false;
    }
    buffer_enabled = true;
    return true;
}

void AP_InertialSensor_SC7U22IIO::disable()
{
    if (buffer_enabled) {
        char path[192];
        snprintf(path, sizeof(path), "%s/buffer/enable", sysfs_path);
        write_text(path, "0");
        buffer_enabled = false;
    }
}

void AP_InertialSensor_SC7U22IIO::start()
{
    const uint32_t id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SPI, 0, 0, DEVTYPE_INS_SC7U22);
    if (!_imu.register_accel(accel_instance, sample_rate_hz, id) ||
        !_imu.register_gyro(gyro_instance, sample_rate_hz, id)) {
        return;
    }
    set_accel_orientation(accel_instance, sensor_rotation);
    set_gyro_orientation(gyro_instance, sensor_rotation);
}

bool AP_InertialSensor_SC7U22IIO::read_scans()
{
    uint8_t scans[scan_size * 64];
    bool received = false;
    for (uint8_t batch = 0; batch < 8; batch++) {
        const ssize_t count = read(fd, scans, sizeof(scans));
        if (count <= 0) {
            if (count < 0 && errno != EAGAIN) {
                _inc_accel_error_count(accel_instance);
                _inc_gyro_error_count(gyro_instance);
            }
            break;
        }
        if (count % scan_size != 0) {
            _inc_accel_error_count(accel_instance);
            _inc_gyro_error_count(gyro_instance);
            break;
        }
        for (ssize_t offset = 0; offset < count; offset += scan_size) {
            const uint8_t *scan = &scans[offset];
            const int64_t timestamp_ns = read_le64(scan + 16);
            if (timestamp_ns <= 0) {
                continue;
            }
            const uint64_t timestamp_us = uint64_t(timestamp_ns) / 1000;
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            const uint64_t now_us = uint64_t(now.tv_sec) * 1000000 + now.tv_nsec / 1000;
            if (timestamp_us > now_us + 5000 || now_us > timestamp_us + 1000000 ||
                (last_timestamp_us != 0 && timestamp_us <= last_timestamp_us)) {
                _inc_accel_error_count(accel_instance);
                _inc_gyro_error_count(gyro_instance);
                last_timestamp_us = timestamp_us;
                continue;
            }
            if (last_timestamp_us != 0 && timestamp_us - last_timestamp_us > 925) {
                _inc_accel_error_count(accel_instance);
                _inc_gyro_error_count(gyro_instance);
                last_timestamp_us = timestamp_us;
                continue;
            }
            last_timestamp_us = timestamp_us;
            constexpr float accel_scale = 16.0f * GRAVITY_MSS / 32768.0f;
            constexpr float gyro_scale = radians(2000.0f) / 32768.0f;
            Vector3f accel(read_le16(scan), read_le16(scan + 2), read_le16(scan + 4));
            Vector3f gyro(read_le16(scan + 6), read_le16(scan + 8), read_le16(scan + 10));
            accel *= accel_scale;
            gyro *= gyro_scale;
            _rotate_and_correct_accel(accel_instance, accel);
            _rotate_and_correct_gyro(gyro_instance, gyro);
            _notify_new_accel_raw_sample(accel_instance, accel);
            _notify_new_gyro_raw_sample(gyro_instance, gyro);
            received = true;
        }
        if (static_cast<size_t>(count) < sizeof(scans)) {
            break;
        }
    }
    return received;
}

bool AP_InertialSensor_SC7U22IIO::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

void AP_InertialSensor_SC7U22IIO::accumulate()
{
    read_scans();
}

#endif // HAL_BOARD_SUBTYPE_LINUX_POWERFIN
