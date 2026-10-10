// Termination paths of ContinuousDataWriter:
//   - a recording killed mid-write leaves no partial CSV row once recovered,
//   - a crash (fatal signal) cuts the CSV back to its last complete row,
//   - SIGINT stops gracefully and produces points.laz,
//   - a leftover points_temp.bin is converted to LAZ on the next start,
//   - the CSV row format is unchanged.
#include "datawriter.h"
#include "packet_replay.h"

#include <cmath>
#include <cstdlib>
#include <random>
#include <sstream>
#include <sys/wait.h>

namespace fs = std::filesystem;

namespace
{
int g_failures = 0;

#define CHECK(cond, ...)                                       \
    do                                                         \
    {                                                          \
        if (!(cond))                                           \
        {                                                      \
            g_failures++;                                      \
            printf("  FAIL %s:%d: %s ", __FILE__, __LINE__, #cond); \
            printf(__VA_ARGS__);                               \
            printf("\n");                                      \
        }                                                      \
    } while (0)

std::string MakeTempDir()
{
    std::string tmpl = (fs::temp_directory_path() / "dlidar_test_XXXXXX").string();
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    char *dir = mkdtemp(buf.data());
    if (!dir)
    {
        perror("mkdtemp");
        exit(2);
    }
    return dir;
}

std::string ReadFile(const std::string &path)
{
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

OutputImuData Imu(int64_t i)
{
    OutputImuData d{};
    d.GyroX = 0.001f * (float)i;
    d.GyroY = -0.25f;
    d.GyroZ = 3.14159f;
    d.AccelerationX = 0.01f;
    d.AccelerationY = -0.98765f;
    d.AccelerationZ = 1.0f / 3.0f;
    d.ImuId = 0;
    d.LidarTimestamp = 1194700 * i;
    d.EpochTimestamp = 1759300000000000000LL + d.LidarTimestamp;
    return d;
}

PointDLidar Point(int64_t i)
{
    PointDLidar p{};
    p.x = 0.001f * (float)(i % 10000);
    p.y = -2.5f + 0.0001f * (float)(i % 777);
    p.z = 1.25f;
    p.intensity = (float)(i % 256);
    p.time = 1e-5 * (double)i;
    p.ring = 1;
    return p;
}

SensorChunk Chunk(int index, int64_t &imu_next, int64_t &point_next, int imu_rows, int points)
{
    SensorChunk c;
    c.chunk_index = index;
    for (int i = 0; i < imu_rows; i++)
        c.imu_data.push_back(Imu(imu_next++));
    for (int i = 0; i < points; i++)
        c.points.push_back(Point(point_next++));
    return c;
}

// Every line ends with '\n', the first is the header, the rest have 9
// numeric fields with strictly increasing timestamps.
bool CsvIsClean(const std::string &path, size_t *rows_out, std::string *why)
{
    std::string text = ReadFile(path);
    if (text.empty() || text.back() != '\n')
    {
        *why = "missing trailing newline / partial last line";
        return false;
    }
    std::istringstream in(text);
    std::string line;
    std::getline(in, line);
    if (line + "\n" != kImuCsvHeader)
    {
        *why = "bad header";
        return false;
    }
    size_t rows = 0;
    long long last = -1;
    while (std::getline(in, line))
    {
        std::istringstream fields(line);
        std::string f;
        std::vector<std::string> v;
        while (fields >> f)
            v.push_back(f);
        if (v.size() != 9)
        {
            *why = "row " + std::to_string(rows) + " has " + std::to_string(v.size()) + " fields: " + line;
            return false;
        }
        long long ts = std::stoll(v[7]);
        if (ts <= last)
        {
            *why = "timestamps not increasing at row " + std::to_string(rows);
            return false;
        }
        last = ts;
        rows++;
    }
    *rows_out = rows;
    return true;
}

// Reads lidar0001.laz in dir and checks it against Point(i).
uint64_t CheckLaz(const std::string &dir, uint64_t expected_total)
{
    uint64_t index = 0;
    {
        std::string path = JoinPath(dir, LazName(1));
        CHECK(fs::exists(path), "no %s", path.c_str());
        laszip_POINTER reader;
        laszip_create(&reader);
        laszip_BOOL compressed = 0;
        CHECK(!laszip_open_reader(reader, path.c_str(), &compressed), "%s", path.c_str());
        laszip_header *header;
        laszip_get_header_pointer(reader, &header);
        laszip_point *pt;
        laszip_get_point_pointer(reader, &pt);
        CHECK(compressed, "%s not compressed", path.c_str());
        for (laszip_U32 i = 0; i < header->number_of_point_records; i++, index++)
        {
            laszip_read_point(reader);
            laszip_F64 c[3];
            laszip_get_coordinates(reader, c);
            PointDLidar e = Point((int64_t)index);
            bool ok = std::fabs(c[0] - e.x) < 1e-4 && std::fabs(c[1] - e.y) < 1e-4 && std::fabs(c[2] - e.z) < 1e-4 &&
                      pt->gps_time == e.time && pt->intensity == (laszip_U16)e.intensity;
            if (!ok)
            {
                CHECK(ok, "point %llu of %s differs", (unsigned long long)index, path.c_str());
                break;
            }
        }
        laszip_close_reader(reader);
        laszip_destroy(reader);
    }
    CHECK(index == expected_total, "LAZ has %llu points, expected %llu", (unsigned long long)index,
          (unsigned long long)expected_total);
    return index;
}

void TestRowFormatUnchanged()
{
    printf("row format unchanged\n");
    OutputImuData d = Imu(12345);
    std::ostringstream old;
    old << std::fixed << std::setprecision(17)
        << d.GyroX << " " << d.GyroY << " " << d.GyroZ << " "
        << d.AccelerationX << " " << d.AccelerationY << " " << d.AccelerationZ << " "
        << d.ImuId << " " << d.LidarTimestamp << " " << d.EpochTimestamp << "\n";
    std::string row;
    AppendImuRow(row, d);
    CHECK(row == old.str(), "\n    new: %s    old: %s", row.c_str(), old.str().c_str());
}

// A child records in a loop and is SIGKILLed at a random moment. The next
// start (RecoverInterruptedRecording) must leave a clean CSV and LAZ.
void TestKilledMidWrite()
{
    printf("killed mid-write -> recovered on next start\n");
    std::mt19937 rng(7);
    int partial_before_recovery = 0;
    for (int run = 0; run < 12; run++)
    {
        std::string dir = MakeTempDir();
        fflush(stdout);
        pid_t pid = fork();
        if (pid == 0)
        {
            ContinuousDataWriter w;
            if (!w.Open(dir))
                _exit(3);
            int64_t imu = 0, pts = 0;
            for (int c = 0;; c++)
                w.AppendChunk(Chunk(c, imu, pts, 3000, 5000));
        }
        usleep(20000 + rng() % 150000);
        kill(pid, SIGKILL);
        int status;
        waitpid(pid, &status, 0);

        std::string text = ReadFile(JoinPath(dir, ImuCsvName(1)));
        partial_before_recovery += !text.empty() && text.back() != '\n';
        uint64_t temp_bytes = fs::file_size(JoinPath(dir, PointsTempName(1)));

        CHECK(RecoverInterruptedRecording(dir, false), "run %d", run);
        size_t rows = 0;
        std::string why;
        CHECK(CsvIsClean(JoinPath(dir, ImuCsvName(1)), &rows, &why), "run %d: %s", run, why.c_str());
        CHECK(!fs::exists(JoinPath(dir, PointsTempName(1))), "temp file left in run %d", run);
        CheckLaz(dir, temp_bytes / sizeof(PointDLidar));
        fs::remove_all(dir);
    }
    printf("  (%d of 12 kills had left a partial row before recovery)\n", partial_before_recovery);
}

// A crash while a row is half written: the fatal-signal handler cuts the
// CSV back to the last complete row before the process dies.
void TestCrashTruncatesCsv()
{
    printf("crash (SIGABRT) with a half-written row -> handler truncates\n");
    std::string dir = MakeTempDir();
    fflush(stdout);
    pid_t pid = fork();
    if (pid == 0)
    {
        InstallSignalHandlers();
        ContinuousDataWriter w;
        if (!w.Open(dir))
            _exit(3);
        int64_t imu = 0, pts = 0;
        for (int c = 0; c < 3; c++)
            w.AppendChunk(Chunk(c, imu, pts, 100, 10));
        const char partial[] = "0.00100000004749745 -0.25000";
        ssize_t ignored = write(CrashTruncateFd().load(), partial, sizeof partial - 1);
        (void)ignored;
        abort();
    }
    int status;
    waitpid(pid, &status, 0);
    CHECK(WIFSIGNALED(status) && WTERMSIG(status) == SIGABRT, "child did not abort");
    size_t rows = 0;
    std::string why;
    CHECK(CsvIsClean(JoinPath(dir, ImuCsvName(1)), &rows, &why), "%s", why.c_str());
    CHECK(rows == 300, "%zu rows", rows);

    // Points of the crashed recording are finalized on the next start, in
    // their own directory so the next recording cannot overwrite them.
    CHECK(RecoverInterruptedRecording(dir, true), "recovery");
    fs::path recovered;
    for (const auto &e : fs::directory_iterator(dir))
        if (e.is_directory() && e.path().filename().string().rfind("recovered_", 0) == 0)
            recovered = e.path();
    CHECK(!recovered.empty(), "no recovered_* directory");
    if (!recovered.empty())
    {
        CHECK(CsvIsClean((recovered / ImuCsvName(1)).string(), &rows, &why) && rows == 300, "%s", why.c_str());
        CheckLaz(recovered.string(), 30);
    }
    CHECK(!fs::exists(JoinPath(dir, PointsTempName(1))), "temp file left");
    CHECK(!fs::exists(JoinPath(dir, ImuCsvName(1))), "CSV not moved");
    fs::remove_all(dir);
}

// SIGINT ends the recording through the normal path: LAZ written, no temp.
void TestSigintFinalizes()
{
    printf("SIGINT -> graceful stop, LAZ written\n");
    std::string dir = MakeTempDir();
    fflush(stdout);
    pid_t pid = fork();
    if (pid == 0)
    {
        InstallSignalHandlers();
        ContinuousDataWriter w;
        if (!w.Open(dir))
            _exit(3);
        int64_t imu = 0, pts = 0;
        for (int c = 0; !StopRequested(); c++)
            w.AppendChunk(Chunk(c, imu, pts, 200, 2000));
        _exit(w.CloseAndPackageLAZ() ? 0 : 4);
    }
    usleep(150000);
    kill(pid, SIGINT);
    int status;
    waitpid(pid, &status, 0);
    CHECK(WIFEXITED(status) && WEXITSTATUS(status) == 0, "child status %d", status);
    size_t rows = 0;
    std::string why;
    CHECK(CsvIsClean(JoinPath(dir, ImuCsvName(1)), &rows, &why), "%s", why.c_str());
    CHECK(!fs::exists(JoinPath(dir, PointsTempName(1))), "temp file left");
    CHECK(fs::exists(JoinPath(dir, LazName(1))), "no lidar0001.laz");
    CheckLaz(dir, rows / 200 * 2000);
    fs::remove_all(dir);
}

// A temp file left by an earlier build/crash, with a partial last point and
// a partial CSV row, is converted to lidar0001.laz on the next start.
void TestLeftoverTempConverted()
{
    printf("leftover points_temp0001.bin -> lidar0001.laz on next start\n");
    std::string dir = MakeTempDir();
    {
        std::ofstream bin(JoinPath(dir, PointsTempName(1)), std::ios::binary);
        for (int64_t i = 0; i < 25000; i++)
        {
            PointDLidar p = Point(i);
            bin.write(reinterpret_cast<const char *>(&p), sizeof p);
        }
        bin.write("partial-point", 13);
        std::ofstream csv(JoinPath(dir, ImuCsvName(1)), std::ios::binary);
        std::string rows = kImuCsvHeader;
        for (int i = 0; i < 10; i++)
            AppendImuRow(rows, Imu(i));
        csv << rows << "0.0010000000474974 -0.2500000000";
    }
    CHECK(sizeof(PointDLidar) == 32, "PointDLidar is %zu bytes", sizeof(PointDLidar));

    CHECK(RecoverInterruptedRecording(dir, false), "recovery");
    size_t rows = 0;
    std::string why;
    CHECK(CsvIsClean(JoinPath(dir, ImuCsvName(1)), &rows, &why) && rows == 10, "%s rows %zu", why.c_str(), rows);
    CHECK(fs::exists(JoinPath(dir, LazName(1))), "missing lidar0001.laz");
    CHECK(!fs::exists(JoinPath(dir, PointsTempName(1))), "temp file left");
    CheckLaz(dir, 25000);

    // Nothing left to recover: a second start does nothing.
    CHECK(RecoverInterruptedRecording(dir, true), "second recovery");
    CHECK(fs::exists(JoinPath(dir, ImuCsvName(1))), "CSV moved without a temp file");
    fs::remove_all(dir);
}

// Each recording takes the next free number; its IMU CSV and LAZ share it and
// earlier recordings are never overwritten.
void TestSessionNumbering()
{
    printf("recordings are numbered, imu and lidar files share the number\n");
    std::string dir = MakeTempDir();
    CHECK(NextSessionIndex(dir) == 1, "first session is not 1");
    for (int expect = 1; expect <= 3; expect++)
    {
        ContinuousDataWriter w;
        CHECK(w.Open(dir), "open %d", expect);
        CHECK(w.SessionIndex() == expect, "session %d got %d", expect, w.SessionIndex());
        int64_t imu = 0, pts = 0;
        w.AppendChunk(Chunk(0, imu, pts, 10, 100));
        CHECK(w.CloseAndPackageLAZ(), "close %d", expect);
        CHECK(fs::exists(JoinPath(dir, ImuCsvName(expect))), "missing imu %d", expect);
        CHECK(fs::exists(JoinPath(dir, LazName(expect))), "missing lidar %d", expect);
    }
    CHECK(ImuCsvName(2) == "imu0002.csv" && LazName(2) == "lidar0002.laz", "names");

    // A leftover temp file of session 4 is finalized in place as lidar0004.laz.
    { std::ofstream bin(JoinPath(dir, PointsTempName(4)), std::ios::binary); PointDLidar p = Point(0); bin.write(reinterpret_cast<const char *>(&p), sizeof p); }
    CHECK(NextSessionIndex(dir) == 5, "next after leftover");
    CHECK(RecoverInterruptedRecording(dir, false), "recovery");
    CHECK(fs::exists(JoinPath(dir, LazName(4))), "no lidar0004.laz");
    CHECK(NextSessionIndex(dir) == 5, "next after recovery");
    fs::remove_all(dir);
}

// The packet timing log and the timing report are written next to the IMU
// CSV and LAZ, and a recovered recording takes them along.
void TestTimingFiles()
{
    printf("packetsNNNN.bin and timingNNNN.json written and recovered\n");
    std::string dir = MakeTempDir();
    {
        ContinuousDataWriter w;
        CHECK(w.Open(dir), "open");
        int64_t imu = 0, pts = 0;
        SensorChunk c = Chunk(0, imu, pts, 10, 100);
        for (uint32_t i = 0; i < 25; i++)
            c.packets.push_back({1000 + i, 2000 + i, 3000 + i, i, i % 2 ? kPacketTypeImu : kPacketTypePoints, 1e-5f, 120});
        c.timing_report = "{\n  \"imu_time_source\": \"sensor\"\n}\n";
        w.AppendChunk(c);
        CHECK(w.CloseAndPackageLAZ(), "close");
    }
    std::vector<PacketTiming> packets;
    CHECK(ReadPacketLog(JoinPath(dir, PacketLogName(1)), packets) && packets.size() == 25 && packets[24].sensor_ns == 3024 &&
              packets[1].type == kPacketTypeImu,
          "packet log: %zu records", packets.size());
    CHECK(ReadFile(JoinPath(dir, TimingName(1))).find("\"sensor\"") != std::string::npos, "timing report");
    CHECK(NextSessionIndex(dir) == 2, "next session");

    // Killed recording 2: everything of it moves to recovered_<time>/.
    {
        std::ofstream(JoinPath(dir, PointsTempName(2)), std::ios::binary);
        std::ofstream(JoinPath(dir, ImuCsvName(2))) << kImuCsvHeader;
        std::ofstream(JoinPath(dir, TimingName(2))) << "{}";
        std::ofstream(JoinPath(dir, PacketLogName(2)), std::ios::binary).write(kPacketLogMagic, sizeof kPacketLogMagic);
    }
    CHECK(RecoverInterruptedRecording(dir, true), "recovery");
    fs::path recovered;
    for (const auto &e : fs::directory_iterator(dir))
        if (e.is_directory())
            recovered = e.path();
    CHECK(!recovered.empty() && fs::exists(recovered / TimingName(2)) && fs::exists(recovered / PacketLogName(2)) &&
              fs::exists(recovered / LazName(2)) && !fs::exists(JoinPath(dir, TimingName(2))),
          "recovered files");
    fs::remove_all(dir);
}
} // namespace

int main()
{
    TestRowFormatUnchanged();
    TestKilledMidWrite();
    TestCrashTruncatesCsv();
    TestSigintFinalizes();
    TestLeftoverTempConverted();
    TestSessionNumbering();
    TestTimingFiles();
    printf(g_failures ? "Data writer: %d checks FAILED\n" : "Data writer: all checks passed\n", g_failures);
    return g_failures ? 1 : 0;
}
