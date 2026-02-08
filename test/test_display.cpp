/*
 * Unit tests for Display_Interface, Stdio_Display, and Dual_Display
 */

#include <gtest/gtest.h>

#include "utils/display_interface.hpp"
#include "utils/stdio_display.hpp"
#include "utils/dual_display.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// ============================================================================
// StdoutCapture — RAII redirect of stdout to a temp file
// ============================================================================

class StdoutCapture {
public:
    StdoutCapture() {
        fflush(stdout);
        saved_fd_ = dup(fileno(stdout));
        tmp_ = tmpfile();
        dup2(fileno(tmp_), fileno(stdout));
    }

    ~StdoutCapture() {
        if (saved_fd_ >= 0) {
            restore();
        }
    }

    std::string get() {
        fflush(stdout);
        dup2(saved_fd_, fileno(stdout));
        close(saved_fd_);
        saved_fd_ = -1;

        fseek(tmp_, 0, SEEK_END);
        long sz = ftell(tmp_);
        rewind(tmp_);

        std::string buf(static_cast<size_t>(sz), '\0');
        size_t n = fread(&buf[0], 1, static_cast<size_t>(sz), tmp_);
        buf.resize(n);
        fclose(tmp_);
        tmp_ = nullptr;

        return buf;
    }

    StdoutCapture(const StdoutCapture &) = delete;
    StdoutCapture &operator=(const StdoutCapture &) = delete;

private:
    void restore() {
        fflush(stdout);
        dup2(saved_fd_, fileno(stdout));
        close(saved_fd_);
        saved_fd_ = -1;
        if (tmp_) {
            fclose(tmp_);
            tmp_ = nullptr;
        }
    }

    int saved_fd_ = -1;
    FILE *tmp_ = nullptr;
};

// ============================================================================
// RecordingDisplay — mock that records every call for Dual_Display tests
// ============================================================================

enum class CallType { Status, Error, Progress, Clear };

struct DisplayCall {
    CallType type;
    std::string sensor;
    std::string msg;
    DisplaySeverity severity = DisplaySeverity::Info;
    uint8_t pct = 0;
};

class RecordingDisplay : public Display_Interface {
public:
    std::vector<DisplayCall> calls;

    void show_status(const char *sensor, const char *msg) override {
        calls.push_back({CallType::Status, sensor, msg, DisplaySeverity::Info, 0});
    }

    void show_error(const char *sensor, const char *msg, DisplaySeverity sev) override {
        calls.push_back({CallType::Error, sensor, msg, sev, 0});
    }

    void show_progress(const char *sensor, uint8_t pct) override {
        calls.push_back({CallType::Progress, sensor, "", DisplaySeverity::Info, pct});
    }

    void clear() override {
        calls.push_back({CallType::Clear, "", "", DisplaySeverity::Info, 0});
    }
};

// ============================================================================
// Test Suite: StdioDisplay_ShowStatus
// ============================================================================

TEST(StdioDisplay_ShowStatus, FormatIncludesSensorAndMessage) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_status("IMU", "Ready");
        out = cap.get();
    }
    EXPECT_EQ(out, "[INFO]  IMU: Ready\n");
}

TEST(StdioDisplay_ShowStatus, EmptySensorAndMessage) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_status("", "");
        out = cap.get();
    }
    EXPECT_EQ(out, "[INFO]  : \n");
}

TEST(StdioDisplay_ShowStatus, LongStrings) {
    Stdio_Display d;
    std::string long_sensor(200, 'S');
    std::string long_msg(200, 'M');
    std::string out;
    {
        StdoutCapture cap;
        d.show_status(long_sensor.c_str(), long_msg.c_str());
        out = cap.get();
    }
    std::string expected = "[INFO]  " + long_sensor + ": " + long_msg + "\n";
    EXPECT_EQ(out, expected);
}

TEST(StdioDisplay_ShowStatus, SpecialCharacters) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_status("%s%d%n", "100%%\tnewline\n");
        out = cap.get();
    }
    // printf with %s format treats args as literal strings, not format specifiers
    EXPECT_EQ(out, "[INFO]  %s%d%n: 100%%\tnewline\n\n");
}

TEST(StdioDisplay_ShowStatus, MultipleCallsAppend) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_status("A", "first");
        d.show_status("B", "second");
        out = cap.get();
    }
    EXPECT_EQ(out, "[INFO]  A: first\n[INFO]  B: second\n");
}

// ============================================================================
// Test Suite: StdioDisplay_ShowError
// ============================================================================

TEST(StdioDisplay_ShowError, InfoSeverityLabel) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_error("ENC", "ok", DisplaySeverity::Info);
        out = cap.get();
    }
    EXPECT_EQ(out, "[INFO] ENC: ok\n");
}

TEST(StdioDisplay_ShowError, WarningSeverityLabel) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_error("ENC", "weak", DisplaySeverity::Warning);
        out = cap.get();
    }
    EXPECT_EQ(out, "[WARN] ENC: weak\n");
}

TEST(StdioDisplay_ShowError, ErrorSeverityLabel) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_error("IMU", "fail", DisplaySeverity::Error);
        out = cap.get();
    }
    EXPECT_EQ(out, "[ERROR] IMU: fail\n");
}

TEST(StdioDisplay_ShowError, FatalSeverityLabel) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_error("DEPTH", "dead", DisplaySeverity::Fatal);
        out = cap.get();
    }
    EXPECT_EQ(out, "[FATAL] DEPTH: dead\n");
}

TEST(StdioDisplay_ShowError, AllSeveritiesDistinct) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_error("S", "m", DisplaySeverity::Info);
        d.show_error("S", "m", DisplaySeverity::Warning);
        d.show_error("S", "m", DisplaySeverity::Error);
        d.show_error("S", "m", DisplaySeverity::Fatal);
        out = cap.get();
    }
    EXPECT_NE(std::string::npos, out.find("[INFO]"));
    EXPECT_NE(std::string::npos, out.find("[WARN]"));
    EXPECT_NE(std::string::npos, out.find("[ERROR]"));
    EXPECT_NE(std::string::npos, out.find("[FATAL]"));

    // Verify they're on separate lines with correct labels
    std::string lines[] = {
        "[INFO] S: m\n",
        "[WARN] S: m\n",
        "[ERROR] S: m\n",
        "[FATAL] S: m\n",
    };
    std::string expected;
    for (const auto &line : lines) expected += line;
    EXPECT_EQ(out, expected);
}

// ============================================================================
// Test Suite: StdioDisplay_ShowProgress
// ============================================================================

TEST(StdioDisplay_ShowProgress, ZeroPercent) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_progress("CAL", 0);
        out = cap.get();
    }
    EXPECT_EQ(out, "[INFO]  CAL: 0% complete\n");
}

TEST(StdioDisplay_ShowProgress, HundredPercent) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_progress("CAL", 100);
        out = cap.get();
    }
    EXPECT_EQ(out, "[INFO]  CAL: 100% complete\n");
}

TEST(StdioDisplay_ShowProgress, MaxUint8) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_progress("CAL", 255);
        out = cap.get();
    }
    EXPECT_EQ(out, "[INFO]  CAL: 255% complete\n");
}

TEST(StdioDisplay_ShowProgress, EmptySensor) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.show_progress("", 50);
        out = cap.get();
    }
    EXPECT_EQ(out, "[INFO]  : 50% complete\n");
}

// ============================================================================
// Test Suite: StdioDisplay_Clear
// ============================================================================

TEST(StdioDisplay_Clear, OutputsAnsiClearSequence) {
    Stdio_Display d;
    std::string out;
    {
        StdoutCapture cap;
        d.clear();
        out = cap.get();
    }
    EXPECT_EQ(out, "\033[2J\033[H");
}

// ============================================================================
// Test Suite: DualDisplay_Forwarding
// ============================================================================

TEST(DualDisplay_Forwarding, ShowStatusForwardsToBoth) {
    RecordingDisplay a, b;
    Dual_Display dual(a, b);
    dual.show_status("IMU", "Ready");

    ASSERT_EQ(a.calls.size(), 1u);
    ASSERT_EQ(b.calls.size(), 1u);
    EXPECT_EQ(a.calls[0].type, CallType::Status);
    EXPECT_EQ(a.calls[0].sensor, "IMU");
    EXPECT_EQ(a.calls[0].msg, "Ready");
    EXPECT_EQ(b.calls[0].type, CallType::Status);
    EXPECT_EQ(b.calls[0].sensor, "IMU");
    EXPECT_EQ(b.calls[0].msg, "Ready");
}

TEST(DualDisplay_Forwarding, ShowErrorForwardsToBothWithSeverity) {
    RecordingDisplay a, b;
    Dual_Display dual(a, b);
    dual.show_error("ENC", "fail", DisplaySeverity::Error);

    ASSERT_EQ(a.calls.size(), 1u);
    ASSERT_EQ(b.calls.size(), 1u);
    EXPECT_EQ(a.calls[0].type, CallType::Error);
    EXPECT_EQ(a.calls[0].severity, DisplaySeverity::Error);
    EXPECT_EQ(b.calls[0].severity, DisplaySeverity::Error);
}

TEST(DualDisplay_Forwarding, ShowProgressForwardsToBothWithPct) {
    RecordingDisplay a, b;
    Dual_Display dual(a, b);
    dual.show_progress("DEPTH", 75);

    ASSERT_EQ(a.calls.size(), 1u);
    ASSERT_EQ(b.calls.size(), 1u);
    EXPECT_EQ(a.calls[0].type, CallType::Progress);
    EXPECT_EQ(a.calls[0].pct, 75);
    EXPECT_EQ(b.calls[0].pct, 75);
}

TEST(DualDisplay_Forwarding, ClearForwardsToBoth) {
    RecordingDisplay a, b;
    Dual_Display dual(a, b);
    dual.clear();

    ASSERT_EQ(a.calls.size(), 1u);
    ASSERT_EQ(b.calls.size(), 1u);
    EXPECT_EQ(a.calls[0].type, CallType::Clear);
    EXPECT_EQ(b.calls[0].type, CallType::Clear);
}

TEST(DualDisplay_Forwarding, MultipleCallsMaintainOrder) {
    RecordingDisplay a, b;
    Dual_Display dual(a, b);

    dual.show_status("A", "1");
    dual.show_error("B", "2", DisplaySeverity::Warning);
    dual.show_progress("C", 50);
    dual.clear();

    ASSERT_EQ(a.calls.size(), 4u);
    ASSERT_EQ(b.calls.size(), 4u);

    EXPECT_EQ(a.calls[0].type, CallType::Status);
    EXPECT_EQ(a.calls[1].type, CallType::Error);
    EXPECT_EQ(a.calls[2].type, CallType::Progress);
    EXPECT_EQ(a.calls[3].type, CallType::Clear);

    EXPECT_EQ(b.calls[0].type, CallType::Status);
    EXPECT_EQ(b.calls[1].type, CallType::Error);
    EXPECT_EQ(b.calls[2].type, CallType::Progress);
    EXPECT_EQ(b.calls[3].type, CallType::Clear);

    // Verify argument fidelity across the sequence
    EXPECT_EQ(a.calls[0].sensor, "A");
    EXPECT_EQ(a.calls[1].severity, DisplaySeverity::Warning);
    EXPECT_EQ(a.calls[2].pct, 50);
}

// ============================================================================
// Test Suite: DualDisplay_EdgeCases
// ============================================================================

TEST(DualDisplay_EdgeCases, EmptyStringsForwarded) {
    RecordingDisplay a, b;
    Dual_Display dual(a, b);
    dual.show_status("", "");
    dual.show_error("", "", DisplaySeverity::Info);

    ASSERT_EQ(a.calls.size(), 2u);
    EXPECT_EQ(a.calls[0].sensor, "");
    EXPECT_EQ(a.calls[0].msg, "");
    EXPECT_EQ(a.calls[1].sensor, "");
    EXPECT_EQ(a.calls[1].msg, "");

    ASSERT_EQ(b.calls.size(), 2u);
    EXPECT_EQ(b.calls[0].sensor, "");
    EXPECT_EQ(b.calls[1].sensor, "");
}

TEST(DualDisplay_EdgeCases, BoundaryPctValues) {
    RecordingDisplay a, b;
    Dual_Display dual(a, b);
    dual.show_progress("S", 0);
    dual.show_progress("S", 255);

    ASSERT_EQ(a.calls.size(), 2u);
    EXPECT_EQ(a.calls[0].pct, 0);
    EXPECT_EQ(a.calls[1].pct, 255);
    ASSERT_EQ(b.calls.size(), 2u);
    EXPECT_EQ(b.calls[0].pct, 0);
    EXPECT_EQ(b.calls[1].pct, 255);
}

// ============================================================================
// Test Suite: DisplayInterface_Polymorphism
// ============================================================================

TEST(DisplayInterface_Polymorphism, VirtualDispatchWorks) {
    RecordingDisplay rec;
    Display_Interface &iface = rec;

    iface.show_status("X", "Y");
    iface.show_error("A", "B", DisplaySeverity::Fatal);
    iface.show_progress("C", 42);
    iface.clear();

    ASSERT_EQ(rec.calls.size(), 4u);
    EXPECT_EQ(rec.calls[0].type, CallType::Status);
    EXPECT_EQ(rec.calls[0].sensor, "X");
    EXPECT_EQ(rec.calls[1].type, CallType::Error);
    EXPECT_EQ(rec.calls[1].severity, DisplaySeverity::Fatal);
    EXPECT_EQ(rec.calls[2].type, CallType::Progress);
    EXPECT_EQ(rec.calls[2].pct, 42);
    EXPECT_EQ(rec.calls[3].type, CallType::Clear);
}

// ============================================================================
// Test Suite: DisplayInterface_Destructor
// ============================================================================

TEST(DisplayInterface_Destructor, VirtualDestructorSafe) {
    // Allocate via base pointer, delete via base pointer — must not leak/crash
    Display_Interface *p = new RecordingDisplay();
    p->show_status("test", "alloc");
    delete p;
    // If we get here without crash or ASAN error, the test passes
    SUCCEED();
}
