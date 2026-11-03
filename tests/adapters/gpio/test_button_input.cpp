// ButtonInput with a scripted button line: a short press, a long press
// (reported while held), and stopping while the button is held.
#include "adapters/gpio/button_input.h"
#include "../../check.h"

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>
#include <time.h>

using namespace dlidar::gpio;
using namespace std::chrono_literals;

namespace
{
int64_t Now()
{
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

// Changes are stamped when queued, on CLOCK_MONOTONIC like the kernel's.
class FakeLine : public ButtonLine
{
public:
    void Press(bool pressed)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back({Now(), pressed});
    }
    bool Wait(int timeout_ms, std::vector<ButtonChange> &out) override
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(std::min(timeout_ms, 10)));
        std::lock_guard<std::mutex> lock(mutex_);
        out.insert(out.end(), queue_.begin(), queue_.end());
        queue_.clear();
        return !fail;
    }
    std::atomic<bool> fail{false};

private:
    std::mutex mutex_;
    std::deque<ButtonChange> queue_;
};

void TestPresses()
{
    auto owned = std::make_unique<FakeLine>();
    FakeLine &line = *owned;
    std::atomic<int> shorts{0}, longs{0};
    ButtonInput button(std::move(owned), [&] { shorts++; }, [&] { longs++; });

    line.Press(true);
    std::this_thread::sleep_for(200ms);
    line.Press(false);
    std::this_thread::sleep_for(300ms);
    CHECK(shorts == 1 && longs == 0, "short press: %d short, %d long", shorts.load(), longs.load());

    line.Press(true);
    std::this_thread::sleep_for(5300ms); // held: reported at 5 s, before the release
    CHECK(longs == 1 && shorts == 1, "long press while held: %d long", longs.load());
    line.Press(false);
    std::this_thread::sleep_for(300ms);
    CHECK(longs == 1 && shorts == 1, "its release reports nothing");

    line.Press(true); // stop while held: the destructor returns quickly
    std::this_thread::sleep_for(100ms);
}

void TestStopAndFailure()
{
    auto owned = std::make_unique<FakeLine>();
    FakeLine &line = *owned;
    auto t0 = std::chrono::steady_clock::now();
    {
        ButtonInput button(std::move(owned), [] {}, [] {});
        line.Press(true);
        std::this_thread::sleep_for(100ms);
    }
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    CHECK(ms < 400, "stopped within %.0f ms while held", ms);

    auto failing = std::make_unique<FakeLine>();
    failing->fail = true;
    ButtonInput button(std::move(failing), [] {}, [] {});
    std::this_thread::sleep_for(100ms);
    CHECK(!button.Running(), "a failed line ends the thread");
}
} // namespace

int main()
{
    TestPresses();
    TestStopAndFailure();
    return FinishTests("Button input");
}
