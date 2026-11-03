#pragma once

#include "app/settings.h"
#include "application/ports/lidar_sensor.h"
#include "domain/recording/recording_profile.h"

#include <memory>
#include <vector>

// The only code that knows every sensor adapter: builds the sensor the
// settings choose. A new sensor is an adapter plus a case here
// (docs/architecture.md, "How to add a sensor").

// What the chosen sensor's recordings say about it.
dlidar::RecordingProfile ProfileFor(const DeviceSettings &settings);

// Every sensor's profile, so a leftover recording of any of them can be finished.
std::vector<dlidar::RecordingProfile> AllProfiles();

// Opens the chosen sensor with its settings; nullptr after printing why.
std::unique_ptr<dlidar::LidarSensor> OpenSensor(const DeviceSettings &settings);
