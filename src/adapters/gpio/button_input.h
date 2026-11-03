#pragma once

#include "adapters/gpio/gpio_lines.h"
#include "adapters/gpio/press_detector.h"

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

namespace dlidar::gpio
{
    // The push button as an input channel: its own thread reads the line,
    // PressDetector turns the changes into presses, and the callbacks run
    // on that thread (they may block, e.g. to wait until a recording is saved).
    class ButtonInput
    {
    public:
        ButtonInput(std::unique_ptr<ButtonLine> line, std::function<void()> on_short, std::function<void()> on_long);
        ~ButtonInput(); // stops the thread (within ~100 ms unless a callback runs)

        // False once the line failed (the thread has ended after a message).
        bool Running() const { return running_; }

    private:
        std::unique_ptr<ButtonLine> line_;
        std::function<void()> on_short_, on_long_;
        PressDetector detector_;
        std::atomic<bool> stop_{false}, running_{true};
        std::thread thread_;

        void Loop();
    };
}
