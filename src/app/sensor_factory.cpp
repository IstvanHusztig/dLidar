#include "app/sensor_factory.h"

#include "adapters/livox_mid360s/livox_packet_state.h"
#include "adapters/unitree_l2/l2_packet_state.h"
#include "adapters/unitree_l2/live_l2_source.h"
#include "adapters/unitree_l2/unitree_l2_sensor.h"

#include <iostream>

dlidar::RecordingProfile ProfileFor(const DeviceSettings &settings)
{
	switch (settings.sensor)
	{
	case SensorKind::LivoxMid360s:
		return dlidar::livox::kRecordingProfile;
	default:
		return dlidar::l2::kRecordingProfile;
	}
}

std::vector<dlidar::RecordingProfile> AllProfiles()
{
	return {dlidar::l2::kRecordingProfile, dlidar::livox::kRecordingProfile};
}

std::unique_ptr<dlidar::LidarSensor> OpenSensor(const DeviceSettings &settings)
{
	switch (settings.sensor)
	{
	case SensorKind::UnitreeL2:
	{
		dlidar::LiveL2Config l2;
		l2.lidar_ip = settings.unitree_l2.lidar_ip;
		l2.lidar_port = settings.unitree_l2.lidar_port;
		l2.local_ip = settings.unitree_l2.local_ip;
		l2.local_port = settings.unitree_l2.local_port;
		l2.work_mode = settings.unitree_l2.work_mode;
		std::unique_ptr<dlidar::LiveL2Source> source = dlidar::LiveL2Source::Open(l2);
		if (!source)
			return nullptr;
		return std::make_unique<dlidar::UnitreeL2Sensor>(std::move(source));
	}
	default:
		std::cerr << "The sensor " << SensorKindName(settings.sensor) << " is not supported by this build." << std::endl;
		return nullptr;
	}
}
