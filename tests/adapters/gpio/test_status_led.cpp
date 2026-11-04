// The status LED: the pattern of every state, the channel driving a fake line
// through a recording and a failure, and a failing line.
#include "adapters/gpio/led_pattern.h"
#include "adapters/gpio/status_led.h"
#include "../../check.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <utility>
#include <thread>
#include <vector>

using namespace dlidar;
using namespace dlidar::gpio;
using namespace std::chrono_literals;

namespace
{
class FakeLed : public LedLine
{
public:
    bool Set(bool on) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        writes.push_back(on);
        level = on;
        return !fail;
    }
    bool Level()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return level;
    }
    // Number of off -> on changes so far.
    int Flashes()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        int n = 0;
        for (size_t i = 1; i < writes.size(); i++)
            n += writes[i] && !writes[i - 1];
        return n + (!writes.empty() && writes[0]);
    }
    std::atomic<bool> fail{false};
    std::vector<bool> writes;
    bool level = false;

private:
    std::mutex mutex_;
};

void TestPattern()
{
    for (int ms : {0, 100, 5000, 600000})
    {
        CHECK(!LedOn(DeviceState::Idle, ms), "idle off at %d", ms);
        CHECK(LedOn(DeviceState::Preparing, ms) && LedOn(DeviceState::Recording, ms), "steady on at %d", ms);
    }
    CHECK(LedOn(DeviceState::Finalizing, 0) && LedOn(DeviceState::Finalizing, 249) && !LedOn(DeviceState::Finalizing, 250) &&
              !LedOn(DeviceState::Finalizing, 499) && LedOn(DeviceState::Finalizing, 500),
          "2 Hz blinking while saving");
    int flashes = 0;
    bool was_on = false;
    for (int ms = 0; ms < 3000; ms++)
    {
        bool on = LedOn(DeviceState::Error, ms);
        flashes += on && !was_on;
        was_on = on;
    }
    CHECK(flashes == 3 && !LedOn(DeviceState::Error, 650) && !LedOn(DeviceState::Error, 60000), "error: %d flashes, then off",
          flashes);
}

// The pairing window: two 100 ms flashes a second while idle, and after the
// failure flashes; recording, saving and failure keep priority.
void TestPairingPattern()
{
    auto flashes = [](DeviceState state, int64_t ms0) {
        int n = 0, on_ms = 0;
        bool was_on = false;
        for (int t = 0; t < 1000; t++)
        {
            bool on = LedOn(state, ms0 + t, true, t);
            n += on && !was_on;
            on_ms += on;
            was_on = on;
        }
        return std::make_pair(n, on_ms);
    };
    CHECK(flashes(DeviceState::Idle, 0) == std::make_pair(2, 200), "idle: two 100 ms flashes a second");
    CHECK(LedOn(DeviceState::Idle, 0, true, 0) && LedOn(DeviceState::Idle, 0, true, 99) &&
              !LedOn(DeviceState::Idle, 0, true, 100) && LedOn(DeviceState::Idle, 0, true, 200) &&
              !LedOn(DeviceState::Idle, 0, true, 300) && !LedOn(DeviceState::Idle, 0, true, 999) &&
              LedOn(DeviceState::Idle, 0, true, 1000),
          "flashes at 0-100 and 200-300 ms of each second");
    for (int t : {0, 150, 500, 999})
    {
        CHECK(LedOn(DeviceState::Preparing, t, true, t) && LedOn(DeviceState::Recording, t, true, t),
              "recording keeps priority at %d", t);
        CHECK(LedOn(DeviceState::Finalizing, t, true, t) == LedOn(DeviceState::Finalizing, t),
              "saving keeps priority at %d", t);
        CHECK(LedOn(DeviceState::Error, t, true, t + 150) == LedOn(DeviceState::Error, t),
              "the failure flashes keep priority at %d", t);
    }
    CHECK(flashes(DeviceState::Error, 600) == std::make_pair(2, 200), "after the failure flashes: the pairing pattern");
    CHECK(!LedOn(DeviceState::Idle, 0, false, 0) && !LedOn(DeviceState::Error, 650, false, 0), "off without the window");
}

DeviceStatus State(DeviceState s)
{
    DeviceStatus st;
    st.state = s;
    return st;
}

void TestChannel()
{
    auto owned = std::make_unique<FakeLed>();
    FakeLed &led = *owned;
    {
        StatusLed channel(std::move(owned));
        std::this_thread::sleep_for(120ms);
        CHECK(!led.Level(), "off while idle");
        channel.OnState(State(DeviceState::Preparing));
        std::this_thread::sleep_for(120ms);
        CHECK(led.Level(), "on within 0.2 s of the start");
        channel.OnState(State(DeviceState::Recording));
        std::this_thread::sleep_for(300ms);
        CHECK(led.Level(), "on while recording");
        int before = led.Flashes();
        channel.OnState(State(DeviceState::Finalizing));
        std::this_thread::sleep_for(1100ms);
        CHECK(led.Flashes() - before >= 2, "blinking while saving: %d", led.Flashes() - before);
        channel.OnState(State(DeviceState::Idle));
        std::this_thread::sleep_for(150ms);
        CHECK(!led.Level(), "off when saved");

        before = led.Flashes();
        channel.OnState(State(DeviceState::Error));
        std::this_thread::sleep_for(1200ms);
        CHECK(led.Flashes() - before == 3 && !led.Level(), "3 flashes then off: %d", led.Flashes() - before);

        before = led.Flashes();
        channel.SetPairing(true);
        std::this_thread::sleep_for(1050ms);
        CHECK(led.Flashes() - before >= 2, "pairing window: flashing after a failure: %d", led.Flashes() - before);
        channel.SetPairing(false);
        std::this_thread::sleep_for(120ms);
        CHECK(!led.Level(), "off when the window closes");
        channel.OnState(State(DeviceState::Recording));
        std::this_thread::sleep_for(120ms);
    }
    CHECK(!led.level, "off when the channel ends");
}

void TestFailingLine()
{
    auto owned = std::make_unique<FakeLed>();
    owned->fail = true;
    StatusLed channel(std::move(owned));
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 1000; i++)
        channel.OnState(State(i % 2 ? DeviceState::Recording : DeviceState::Finalizing));
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::this_thread::sleep_for(300ms);
    CHECK(ms < 50 && channel.WriteFailures() > 0, "OnState never blocks (%.1f ms), %llu failures", ms,
          (unsigned long long)channel.WriteFailures());
}
} // namespace

int main()
{
    TestPattern();
    TestPairingPattern();
    TestChannel();
    TestFailingLine();
    return FinishTests("Status LED");
}
