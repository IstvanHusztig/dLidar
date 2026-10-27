// The status LED: the pattern of every state, the channel driving a fake line
// through a recording and a failure, and a failing line.
#include "adapters/gpio/led_pattern.h"
#include "adapters/gpio/status_led.h"
#include "../../check.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
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
    TestChannel();
    TestFailingLine();
    return FinishTests("Status LED");
}
