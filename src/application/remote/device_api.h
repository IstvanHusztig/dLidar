#pragma once

#include "application/ports/device_control.h"
#include "application/ports/storage_space.h"

#include <cstdint>
#include <optional>
#include <string>

namespace dlidar
{
    // The remote API's protocol version (docs/phone-protocol.md).
    const int kProtocolVersion = 1;

    // What the device is: filled in by the composition root.
    struct DeviceInfo
    {
        std::string version;      // the build (git describe)
        std::string sensor;       // the configured sensor's id: unitree_l2, livox_mid360s
        std::string sensor_model; // its name, as in timingNNNN.json
        int recording_format = 0;
    };

    struct StatusView
    {
        DeviceStatus status;
        std::optional<uint64_t> free_bytes; // empty: unknown
    };

    // The typed commands a remote client (the phone) may give, independent of
    // any transport or message format. Thread-safe like DeviceControl. Phase
    // 3 adds Start() and Stop().
    class DeviceApi
    {
    public:
        // `space` may be null (free space unknown).
        DeviceApi(DeviceControl &control, const StorageSpace *space, DeviceInfo info)
            : control_(control), space_(space), info_(std::move(info)) {}

        StatusView Status() const
        {
            StatusView v;
            v.status = control_.Status();
            if (space_)
                v.free_bytes = space_->FreeBytes();
            return v;
        }

        const DeviceInfo &Info() const { return info_; }

    private:
        DeviceControl &control_;
        const StorageSpace *space_;
        DeviceInfo info_;
    };
}
