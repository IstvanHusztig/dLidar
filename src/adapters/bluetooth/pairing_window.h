#pragma once

#include <memory>

namespace dlidar::bluetooth
{
    // The Bluetooth pairing window: for `seconds` the Pi's adapter is pairable
    // and a NoInputNoOutput agent accepts every pairing (Just Works, no PIN);
    // then the adapter is set not pairable again. Runs BlueZ over D-Bus on its
    // own thread and GMainContext; problems are printed as warnings and never
    // affect a recording. Opened at start by [phone] pairing_at_start, later by
    // the pairing button.
    class PairingWindow
    {
    public:
        PairingWindow();
        ~PairingWindow(); // closes an open window and stops the thread

        // Opens the window (again) for `seconds`. False with a warning printed
        // if BlueZ cannot be reached.
        bool Open(int seconds);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
