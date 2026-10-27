#include "adapters/gpio/gpio_line.h"

#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace dlidar::gpio
{
    namespace
    {
        const char *const kHeaderChips[] = {"pinctrl-rp1", "pinctrl-bcm2835"};

        // Opens the chip of the 40-pin header; -1 with the reason in `error`.
        int OpenHeaderChip(std::string &error)
        {
            std::string denied;
            DIR *dir = opendir("/dev");
            if (!dir)
            {
                error = std::string("cannot list /dev: ") + strerror(errno);
                return -1;
            }
            int found = -1;
            for (const char *label : kHeaderChips)
            {
                rewinddir(dir);
                while (dirent *e = readdir(dir))
                {
                    if (strncmp(e->d_name, "gpiochip", 8) != 0)
                        continue;
                    std::string path = std::string("/dev/") + e->d_name;
                    int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
                    if (fd < 0)
                    {
                        if (errno == EACCES && denied.empty())
                            denied = path;
                        continue;
                    }
                    gpiochip_info info{};
                    if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &info) == 0 && strcmp(info.label, label) == 0)
                    {
                        found = fd;
                        break;
                    }
                    close(fd);
                }
                if (found >= 0)
                    break;
            }
            closedir(dir);
            if (found < 0)
                error = !denied.empty() ? "no permission to open " + denied +
                                              " (add the user to group gpio: sudo usermod -aG gpio $USER, then log in again)"
                                        : std::string("no GPIO chip labelled pinctrl-rp1 or pinctrl-bcm2835 found");
            return found;
        }

        int Request(int chip, int line, uint64_t flags, uint32_t debounce_us, std::string &error)
        {
            gpio_v2_line_request req{};
            req.offsets[0] = (uint32_t)line;
            req.num_lines = 1;
            snprintf(req.consumer, sizeof req.consumer, "dLidar");
            req.config.flags = flags;
            if (debounce_us)
            {
                req.config.num_attrs = 1;
                req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_DEBOUNCE;
                req.config.attrs[0].attr.debounce_period_us = debounce_us;
                req.config.attrs[0].mask = 1;
            }
            if (ioctl(chip, GPIO_V2_GET_LINE_IOCTL, &req) < 0)
            {
                error = "cannot request GPIO " + std::to_string(line) + ": " + strerror(errno) +
                        (errno == EBUSY ? " (the line is used by another program or driver)" : "");
                return -1;
            }
            return req.fd;
        }

        int OpenLine(int line, uint64_t flags, uint32_t debounce_us, std::string &error)
        {
            if (line < 0 || line > 27)
            {
                error = "GPIO " + std::to_string(line) + " is not on the 40-pin header (0 to 27)";
                return -1;
            }
            int chip = OpenHeaderChip(error);
            if (chip < 0)
                return -1;
            int fd = Request(chip, line, flags, debounce_us, error);
            if (fd < 0 && debounce_us && errno == EINVAL) // a kernel without debounce: PressDetector debounces
                fd = Request(chip, line, flags, 0, error);
            close(chip);
            return fd;
        }
    }

    std::unique_ptr<GpioLine> GpioLine::OpenButton(int line, std::string &error)
    {
        int fd = OpenLine(line,
                          GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_BIAS_PULL_UP | GPIO_V2_LINE_FLAG_EDGE_RISING |
                              GPIO_V2_LINE_FLAG_EDGE_FALLING,
                          50000, error);
        if (fd < 0)
            return nullptr;
        return std::unique_ptr<GpioLine>(new GpioLine(fd, true));
    }

    std::unique_ptr<GpioLine> GpioLine::OpenLed(int line, std::string &error)
    {
        int fd = OpenLine(line, GPIO_V2_LINE_FLAG_OUTPUT, 0, error);
        if (fd < 0)
            return nullptr;
        std::unique_ptr<GpioLine> led(new GpioLine(fd, false));
        led->Set(false);
        return led;
    }

    GpioLine::~GpioLine()
    {
        if (!active_low_)
            Set(false); // the LED is off when dLidar is not running
        close(fd_);
    }

    bool GpioLine::Wait(int timeout_ms, std::vector<ButtonChange> &out)
    {
        pollfd p{fd_, POLLIN, 0};
        int r = poll(&p, 1, timeout_ms);
        if (r < 0)
            return errno == EINTR;
        if (r == 0)
            return true;
        gpio_v2_line_event events[16];
        ssize_t n = read(fd_, events, sizeof events);
        if (n < 0)
            return errno == EINTR || errno == EAGAIN;
        for (ssize_t i = 0; i < n / (ssize_t)sizeof events[0]; i++)
        {
            bool high = events[i].id == GPIO_V2_LINE_EVENT_RISING_EDGE;
            out.push_back({(int64_t)events[i].timestamp_ns, active_low_ ? !high : high}); // CLOCK_MONOTONIC
        }
        return true;
    }

    bool GpioLine::Set(bool on)
    {
        gpio_v2_line_values v{};
        v.mask = 1;
        v.bits = (active_low_ ? !on : on) ? 1 : 0;
        return ioctl(fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &v) == 0;
    }
}
