#pragma once

#include <stdint.h>

namespace dlidar::gpio
{
    enum class Press
    {
        None,
        Short, // released within the long-press time
        Long   // held for the long-press time (reported while still held)
    };

    // Turns the raw level changes of a push button into presses. A level
    // counts only once it has held for the debounce time, so contact bounce
    // and short glitches give nothing; a press is timed from the raw change
    // that started it. Pure: fed with times, no clock or I/O.
    class PressDetector
    {
    public:
        explicit PressDetector(int64_t debounce_ns = 50000000, int64_t long_ns = 5000000000LL)
            : debounce_ns_(debounce_ns), long_ns_(long_ns)
        {
        }

        // The raw level changed at `t_ns` (true = pressed).
        Press OnLevel(int64_t t_ns, bool pressed)
        {
            Press p = Update(t_ns);
            if (pressed != raw_)
            {
                raw_ = pressed;
                raw_since_ = t_ns;
            }
            return p;
        }

        // Time passed without a change (call it every ~100 ms).
        Press Tick(int64_t t_ns) { return Update(t_ns); }

        bool Pressed() const { return stable_; }

    private:
        int64_t debounce_ns_, long_ns_;
        bool raw_ = false, stable_ = false, long_fired_ = false;
        int64_t raw_since_ = 0, press_start_ = 0;

        Press Update(int64_t t_ns)
        {
            Press p = Press::None;
            if (raw_ != stable_ && t_ns - raw_since_ >= debounce_ns_)
            {
                stable_ = raw_;
                if (stable_)
                {
                    press_start_ = raw_since_;
                    long_fired_ = false;
                }
                else if (!long_fired_ && raw_since_ - press_start_ < long_ns_)
                    p = Press::Short;
            }
            // Not while a release is being debounced: that press ends short.
            if (p == Press::None && stable_ && raw_ && !long_fired_ && t_ns - press_start_ >= long_ns_)
            {
                long_fired_ = true;
                p = Press::Long;
            }
            return p;
        }
    };
}
