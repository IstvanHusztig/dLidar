#include "app/sensor_factory.h"

#include "adapters/livox_mid360s/live_livox_source.h"
#include "adapters/livox_mid360s/livox_mid360s_sensor.h"
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
	case SensorKind::LivoxMid360s:
	{
		const LivoxMid360sSettings &s = settings.livox_mid360s;
		dlidar::livox::LiveLivoxConfig livox;
		livox.lidar_ip = s.lidar_ip;
		livox.host_ip = s.host_ip;
		livox.cmd_data_port = s.cmd_data_port;
		livox.push_msg_port = s.push_msg_port;
		livox.point_data_port = s.point_data_port;
		livox.imu_data_port = s.imu_data_port;
		livox.log_data_port = s.log_data_port;
		livox.imu_rate_hz = s.imu_rate_hz;
		livox.imu_accel_range_g = s.imu_accel_range_g;
		livox.imu_gyro_range_dps = s.imu_gyro_range_dps;
		std::unique_ptr<dlidar::livox::LiveLivoxSource> source = dlidar::livox::LiveLivoxSource::Open(livox);
		if (!source)
			return nullptr;
		return std::make_unique<dlidar::LivoxMid360sSensor>(std::move(source), s.drop_low_confidence);
	}
	default:
		std::cerr << "The sensor " << SensorKindName(settings.sensor) << " is not supported by this build." << std::endl;
		return nullptr;
	}
}
