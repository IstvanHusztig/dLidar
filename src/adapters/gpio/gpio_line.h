#pragma once

#include "adapters/gpio/gpio_lines.h"

#include <memory>
#include <string>

namespace dlidar::gpio
{
    // One GPIO line of the Pi's 40-pin header through the kernel's GPIO
    // character device (uAPI v2, <linux/gpio.h>); the only code that sees
    // it. The header's lines are on the chip labelled pinctrl-rp1 (Pi 5) or
    // pinctrl-bcm2835 (Pi 4); its /dev/gpiochipN number depends on the kernel.
    class GpioLine : public ButtonLine, public LedLine
    {
    public:
        // A button to GND: input with the internal pull-up, both edges,
        // 50 ms kernel debounce where supported; low = pressed. nullptr with
        // the reason in `error`.
        static std::unique_ptr<GpioLine> OpenButton(int line, std::string &error);
        // An LED to GND: output, active high, starting off.
        static std::unique_ptr<GpioLine> OpenLed(int line, std::string &error);
        ~GpioLine() override;

        bool Wait(int timeout_ms, std::vector<ButtonChange> &out) override;
        bool Set(bool on) override;

    private:
        GpioLine(int fd, bool active_low) : fd_(fd), active_low_(active_low) {}
        int fd_;
        bool active_low_;
    };
}
