#pragma once

#include <functional>
#include <iostream>
#include <utility>

// What the pairing button does (docs/manual.html, "Wiring and pinout";
// openspec phone-link-commands, design D5): a short press opens the
// Bluetooth pairing window for 2 minutes, a 5 s hold first forgets every
// bonded phone. Neither touches a recording. `open` is
// PhoneLink::OpenPairingWindow(forget), empty when the phone link is not
// running (then a press only warns). Runs on the button's thread.
class PairingButtonCommands
{
public:
	explicit PairingButtonCommands(std::function<void(bool forget)> open) : open_(std::move(open)) {}

	void OnShort() { Press(false); }
	void OnLong() { Press(true); }

private:
	std::function<void(bool forget)> open_;

	void Press(bool forget)
	{
		std::cout << (forget ? "pairing button: long press, forgetting every phone"
							 : "pairing button: opening the pairing window")
				  << std::endl;
		if (open_)
			open_(forget);
		else
			std::cerr << "WARNING: no pairing window: the phone link is not running" << std::endl;
	}
};
