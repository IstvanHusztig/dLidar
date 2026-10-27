// LivoxPacketPool, the SDK-callback -> capture-thread bridge: a producer
// thread at the Mid-360S packet rate (2,300 packets/s for 20 s) while the
// consumer stalls twice for 5 s. Every packet comes out once, in order, with
// its arrival time; the pool grows past its initial slots with one warning.
#include "adapters/livox_mid360s/livox_packet_pool.h"
#include "../../check.h"

#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

using namespace dlidar::livox;
using Clock = std::chrono::steady_clock;

namespace
{
void TestBurstsAndStalls()
{
    const int kRate = 2300, kSeconds = 20;
    const uint32_t kTotal = kRate * kSeconds;
    LivoxPacketPool pool;
    std::thread producer([&] {
        std::vector<uint8_t> packet(1380);
        Clock::time_point start = Clock::now();
        for (uint32_t i = 0; i < kTotal; i++)
        {
            std::this_thread::sleep_until(start + std::chrono::microseconds((int64_t)i * 1000000 / kRate));
            memcpy(packet.data(), &i, sizeof i);
            packet[1379] = (uint8_t)i;
            pool.Push(HostTime{(int64_t)i * 1000, (int64_t)i * 1000 + 7}, packet.data(), packet.size() - (i % 3));
        }
    });

    uint32_t expect = 0, bad = 0, warnings = 0;
    Clock::time_point start = Clock::now();
    bool stalled1 = false, stalled2 = false;
    while (expect < kTotal && Clock::now() - start < std::chrono::seconds(kSeconds + 30))
    {
        double t = std::chrono::duration<double>(Clock::now() - start).count();
        if (!stalled1 && t > 4)
            stalled1 = true, std::this_thread::sleep_for(std::chrono::seconds(5));
        if (!stalled2 && t > 12)
            stalled2 = true, std::this_thread::sleep_for(std::chrono::seconds(5));
        if (!pool.Pop(std::chrono::milliseconds(100)))
            continue;
        uint32_t i;
        memcpy(&i, pool.Data(), sizeof i);
        HostTime a = pool.Arrival();
        if (i != expect || a.mono_ns != (int64_t)i * 1000 || a.unix_ns != (int64_t)i * 1000 + 7 || pool.Size() != 1380 - i % 3 ||
            (pool.Size() == 1380 && pool.Data()[1379] != (uint8_t)i))
            bad++;
        expect = i + 1;
        warnings += pool.TakeGrowWarning();
    }
    producer.join();
    CHECK(expect == kTotal && bad == 0, "%u of %u packets in order, %u wrong", expect, kTotal, bad);
    CHECK(pool.Slots() > pool.InitialSlots(), "the pool grew: %zu slots (initial %zu)", pool.Slots(), pool.InitialSlots());
    CHECK(warnings == 1 && !pool.TakeGrowWarning(), "%u grow warnings", warnings);
    CHECK(pool.Queued() == 0 && pool.Oversized() == 0, "queue empty");
    printf("  %u packets, pool grew to %zu slots, one warning\n", kTotal, pool.Slots());
}

void TestClearAndOversized()
{
    LivoxPacketPool pool(4, 2);
    uint8_t b[LivoxPacketPool::kSlotBytes + 1] = {1};
    for (int i = 0; i < 10; i++)
        pool.Push(HostTime{i, i}, b, 100);
    pool.Push(HostTime{}, b, sizeof b);
    CHECK(pool.Oversized() == 1 && pool.Queued() == 10, "oversized %llu queued %zu", (unsigned long long)pool.Oversized(),
          pool.Queued());
    CHECK(pool.Pop(std::chrono::milliseconds(0)) && pool.Arrival().mono_ns == 0, "first packet");
    CHECK(pool.Clear() == 9 && pool.Queued() == 0, "the backlog is dropped");
    CHECK(!pool.Pop(std::chrono::milliseconds(10)), "nothing after a clear");
    size_t slots = pool.Slots();
    for (int i = 0; i < 10; i++)
        pool.Push(HostTime{i, i}, b, 100);
    CHECK(pool.Slots() == slots, "freed slots are reused: %zu -> %zu", slots, pool.Slots());
}
// Between recordings: pushes are dropped (counted), turning accepting off
// drops the queue, and the pool does not grow however much arrives.
void TestIdle()
{
    LivoxPacketPool pool(4, 2);
    uint8_t b[100] = {1};
    for (int i = 0; i < 3; i++)
        pool.Push(HostTime{i, i}, b, sizeof b);
    pool.SetAccepting(false);
    CHECK(pool.Queued() == 0 && !pool.Accepting(), "stop drops the queue: %zu queued", pool.Queued());
    size_t slots = pool.Slots();
    for (int i = 0; i < 23000; i++) // 10 s of Mid-360S packets
        pool.Push(HostTime{i, i}, b, sizeof b);
    CHECK(pool.DroppedIdle() == 23000 && pool.Queued() == 0 && pool.Slots() == slots && !pool.TakeGrowWarning(),
          "idle: %llu dropped, %zu queued, %zu slots", (unsigned long long)pool.DroppedIdle(), pool.Queued(), pool.Slots());
    pool.SetAccepting(true);
    pool.Push(HostTime{7, 7}, b, sizeof b);
    CHECK(pool.Pop(std::chrono::milliseconds(0)) && pool.Arrival().mono_ns == 7, "accepted again after Prepare");
}
} // namespace

int main()
{
    TestClearAndOversized();
    TestIdle();
    TestBurstsAndStalls();
    return FinishTests("Livox packet pool");
}
