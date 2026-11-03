// LineFramer (openspec phone-link-core, specs/phone-api "Newline-delimited
// JSON messages"): pieces, several lines per feed, the 4096-byte limit
// including the "\n", recovery after a too-large line, reset.
#include "adapters/phone/line_framer.h"
#include "../../check.h"

#include <string>
#include <vector>

using namespace dlidar::phone;

namespace
{
struct Sink
{
    std::vector<std::string> lines;
    int too_large = 0;
    void Feed(LineFramer &f, const std::string &data)
    {
        f.Feed(data, [&](std::string_view l) { lines.emplace_back(l); }, [&] { too_large++; });
    }
};

void TestPieces()
{
    printf("a line split across 3 feeds, 2 lines in one feed\n");
    LineFramer f;
    Sink s;
    s.Feed(f, "{\"v\":1,");
    s.Feed(f, "\"id\":7,");
    CHECK(s.lines.empty(), "no line before the newline");
    s.Feed(f, "\"cmd\":\"hello\"}\n");
    CHECK(s.lines.size() == 1 && s.lines[0] == "{\"v\":1,\"id\":7,\"cmd\":\"hello\"}", "joined line");
    s.Feed(f, "a\nb\nc");
    CHECK(s.lines.size() == 3 && s.lines[1] == "a" && s.lines[2] == "b", "two lines in one feed");
    s.Feed(f, "\n");
    CHECK(s.lines.size() == 4 && s.lines[3] == "c", "rest completed");
    CHECK(s.too_large == 0, "nothing too large");
}

void TestLimit()
{
    printf("4096 bytes including the newline accepted, 4097 too large once\n");
    LineFramer f;
    Sink s;
    s.Feed(f, std::string(4095, 'x') + "\n");
    CHECK(s.lines.size() == 1 && s.lines[0].size() == 4095 && s.too_large == 0, "4096 accepted");

    // 4097 bytes, fed in pieces: reported once, then the next line is read.
    s.Feed(f, std::string(3000, 'y'));
    s.Feed(f, std::string(1000, 'y'));
    CHECK(s.too_large == 0, "4000 bytes so far is fine");
    s.Feed(f, std::string(96, 'y'));
    CHECK(s.too_large == 1, "4096 bytes without a newline is too large");
    s.Feed(f, std::string(5000, 'y'));
    s.Feed(f, "yy\nnext\n");
    CHECK(s.too_large == 1, "reported once");
    CHECK(s.lines.size() == 2 && s.lines[1] == "next", "next line recovered");

    Sink t;
    LineFramer g;
    t.Feed(g, std::string(4096, 'z') + "\nok\n");
    CHECK(t.too_large == 1 && t.lines.size() == 1 && t.lines[0] == "ok", "4097 in one feed");
}

void TestReset()
{
    printf("reset drops a partial line\n");
    LineFramer f;
    Sink s;
    s.Feed(f, "partial");
    f.Reset();
    s.Feed(f, "whole\n");
    CHECK(s.lines.size() == 1 && s.lines[0] == "whole", "after reset");

    s.Feed(f, std::string(5000, 'x'));
    f.Reset();
    s.Feed(f, "again\n");
    CHECK(s.lines.size() == 2 && s.lines[1] == "again", "reset ends discarding");
}
} // namespace

int main()
{
    TestPieces();
    TestLimit();
    TestReset();
    return FinishTests("line_framer");
}
