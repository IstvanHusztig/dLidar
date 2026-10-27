#pragma once

#include "application/device/scanner_controller.h"

#include <functional>
#include <iostream>
#include <utility>

// What the push button does (docs/manual.html, "Wiring and pinout"): a short
// press starts a recording (Idle/Error) or stops it (Preparing/Recording) and
// is ignored while one is being saved; a long press stops any recording,
// waits until it is saved and powers the Pi down. Runs on the button's thread.
class ButtonCommands
{
public:
	// `power_off` returns false if the power-down was refused.
	ButtonCommands(dlidar::ScannerController &controller, std::function<bool()> power_off)
		: controller_(controller), power_off_(std::move(power_off))
	{
	}

	void OnShort()
	{
		switch (controller_.Status().state)
		{
		case dlidar::DeviceState::Idle:
		case dlidar::DeviceState::Error:
			std::cout << "button: start" << std::endl;
			controller_.StartRecording();
			break;
		case dlidar::DeviceState::Preparing:
		case dlidar::DeviceState::Recording:
			std::cout << "button: stop" << std::endl;
			controller_.StopRecording();
			break;
		case dlidar::DeviceState::Finalizing:
			std::cout << "button: press ignored, the recording is being saved" << std::endl;
			break;
		}
	}

	void OnLong()
	{
		std::cout << "button: long press, saving and powering down" << std::endl;
		controller_.StopRecording();
		controller_.WaitIdle();
		if (!power_off_())
			std::cerr << "WARNING: the power-down was refused; install systemd/50-dlidar-poweroff.rules "
						 "(docs/manual.html, \"Running as a service\"). dLidar keeps running."
					  << std::endl;
	}

private:
	dlidar::ScannerController &controller_;
	std::function<bool()> power_off_;
};
