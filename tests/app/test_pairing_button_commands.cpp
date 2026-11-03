// PairingButtonCommands: a short press opens the pairing window, a long press
// forgets every phone first, and without a phone link a press only warns.
#include "app/pairing_button_commands.h"
#include "../check.h"

#include <sstream>
#include <string>
#include <vector>

namespace
{
// Captures std::cout and std::cerr while it lives.
struct Capture
{
	std::ostringstream out, err;
	std::streambuf *old_out = std::cout.rdbuf(out.rdbuf()), *old_err = std::cerr.rdbuf(err.rdbuf());
	~Capture()
	{
		std::cout.rdbuf(old_out);
		std::cerr.rdbuf(old_err);
	}
};

void TestPresses()
{
	std::vector<bool> calls;
	PairingButtonCommands button([&](bool forget) { calls.push_back(forget); });
	std::string out, err;
	{
		Capture c;
		button.OnShort();
		button.OnLong();
		out = c.out.str();
		err = c.err.str();
	}
	CHECK(calls == std::vector<bool>({false, true}), "short opens, long forgets then opens (%zu calls)", calls.size());
	CHECK(out == "pairing button: opening the pairing window\npairing button: long press, forgetting every phone\n",
		  "log lines: %s", out.c_str());
	CHECK(err.empty(), "no warning: %s", err.c_str());
}

void TestNoPhoneLink()
{
	PairingButtonCommands button(nullptr);
	std::string err;
	{
		Capture c;
		button.OnShort();
		button.OnLong();
		err = c.err.str();
	}
	CHECK(err == "WARNING: no pairing window: the phone link is not running\n"
				 "WARNING: no pairing window: the phone link is not running\n",
		  "a warning per press: %s", err.c_str());
}
} // namespace

int main()
{
	TestPresses();
	TestNoPhoneLink();
	return FinishTests("pairing_button_commands");
}
