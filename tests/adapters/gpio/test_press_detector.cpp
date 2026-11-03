// PressDetector: debounce, short and long presses, from timed level changes.
#include "adapters/gpio/press_detector.h"
#include "../../check.h"

#include <vector>

using namespace dlidar::gpio;

namespace
{
const int64_t MS = 1000000;

// Feeds (ms, level) changes, then ticks every 10 ms up to `end_ms`; returns
// the presses with the time they were reported.
std::vector<std::pair<int64_t, Press>> Run(const std::vector<std::pair<int64_t, bool>> &changes, int64_t end_ms)
{
    PressDetector d;
    std::vector<std::pair<int64_t, Press>> out;
    size_t next = 0;
    for (int64_t t = 0; t <= end_ms; t++)
    {
        while (next < changes.size() && changes[next].first == t)
        {
            Press p = d.OnLevel(t * MS, changes[next].second);
            if (p != Press::None)
                out.push_back({t, p});
            next++;
        }
        if (t % 10 == 0)
        {
            Press p = d.Tick(t * MS);
            if (p != Press::None)
                out.push_back({t, p});
        }
    }
    return out;
}

void TestPresses()
{
    auto r = Run({{100, true}, {400, false}}, 1000);
    CHECK(r.size() == 1 && r[0].second == Press::Short && r[0].first >= 450 && r[0].first <= 460, "clean short press");

    r = Run({{100, true}, {103, false}, {108, true}, {112, false}, {119, true}, {500, false}, {504, true}, {509, false}},
            1500);
    CHECK(r.size() == 1 && r[0].second == Press::Short, "bounces on press and release give one press (%zu)", r.size());

    r = Run({{100, true}, {130, false}}, 1000);
    CHECK(r.empty(), "a 30 ms glitch gives nothing (%zu)", r.size());

    r = Run({{1000, true}, {9000, false}}, 10000);
    CHECK(r.size() == 1 && r[0].second == Press::Long && r[0].first == 6000, "long press at 5.0 s held, while held: %lld",
          r.empty() ? -1LL : (long long)r[0].first);

    r = Run({{1000, true}, {5990, false}}, 7000);
    CHECK(r.size() == 1 && r[0].second == Press::Short, "released at 4.99 s: short");

    r = Run({{100, true}, {300, false}, {400, true}, {600, false}}, 1000);
    CHECK(r.size() == 2 && r[0].second == Press::Short && r[1].second == Press::Short, "two presses 100 ms apart (%zu)",
          r.size());

    r = Run({{100, true}, {6000, false}, {6500, true}, {6700, false}}, 8000);
    CHECK(r.size() == 2 && r[0].second == Press::Long && r[1].second == Press::Short,
          "after a long press the release reports nothing, the next press works");
}
} // namespace

int main()
{
    TestPresses();
    return FinishTests("Press detector");
}
