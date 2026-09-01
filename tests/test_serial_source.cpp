#include "wbr_gps/serial_source.hpp"
#include "wbr_gps/state_store.hpp"
#include "pty_harness.h"
#include "test_util.h"

#include <poll.h>
#include <vector>

// Drain whatever the source can read right now.
static void pump(wbr_gps::SerialSource& src, wbr_gps::StateStore& st, int64_t now,
                 std::vector<std::string>* lines = nullptr)
{
    struct pollfd p { src.fd(), POLLIN, 0 };
    while (poll(&p, 1, 50) > 0 && (p.revents & POLLIN)) {
        src.on_readable(st, now, [&](const std::string& l) {
            if (lines) lines->push_back(l);
        });
        p.revents = 0;
    }
}

static void test_reads_complete_sentences()
{
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());

    wbr_gps::SerialSource src;
    src.set_path(pty.slave_path);
    ASSERT_TRUE(src.open_device());

    wbr_gps::StateStore st;
    std::vector<std::string> lines;

    pty.emit("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F");
    pump(src, st, 1000, &lines);

    ASSERT_EQ(lines.size(), (size_t)1);
    ASSERT_TRUE(st.current().has_fix);
    ASSERT_NEAR(st.current().lat, 13.0028260, 1e-6);
}

static void test_reassembles_split_writes()
{
    // A sentence arriving in three chunks must still parse. This is the case
    // the old per-byte readers got right by accident and the GUI got wrong.
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    wbr_gps::SerialSource src;
    src.set_path(pty.slave_path);
    ASSERT_TRUE(src.open_device());
    wbr_gps::StateStore st;

    pty.emit_raw("$GNGGA,045519.50,1300.16956,N,");
    pump(src, st, 1000);
    ASSERT_FALSE(st.current().has_fix);          // incomplete: nothing applied

    pty.emit_raw("07740.79521,E,1,04,1.33,921.8,");
    pump(src, st, 1000);
    ASSERT_FALSE(st.current().has_fix);

    pty.emit_raw("M,-86.3,M,,*6F\r\n");
    pump(src, st, 1000);
    ASSERT_TRUE(st.current().has_fix);
}

static void test_oversized_line_is_discarded()
{
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    wbr_gps::SerialSource src;
    src.set_path(pty.slave_path);
    ASSERT_TRUE(src.open_device());
    wbr_gps::StateStore st;

    // 4 KB with no newline must not grow the buffer without bound.
    pty.emit_raw(std::string(4096, 'A'));
    pump(src, st, 1000);
    // Then a good sentence must still parse: the buffer recovered.
    pty.emit("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F");
    pump(src, st, 1000);
    ASSERT_TRUE(st.current().has_fix);
}

static void test_torn_input_rejected_and_counted()
{
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    wbr_gps::SerialSource src;
    src.set_path(pty.slave_path);
    ASSERT_TRUE(src.open_device());
    wbr_gps::StateStore st;

    pty.emit(",,3,,5$G32,,,9151,,2,96*G3,,1,,,8,,6379$M055A3.5N7.500,*");
    pump(src, st, 1000);
    ASSERT_FALSE(st.current().has_fix);
    ASSERT_EQ(st.rejected_count(), (uint64_t)1);
}

static void test_open_failure_and_backoff()
{
    wbr_gps::SerialSource src;
    src.set_path("/dev/definitely-not-a-device");
    ASSERT_FALSE(src.open_device());
    ASSERT_EQ(src.fd(), -1);

    // Backoff must start short and grow, and must be capped at 5 s.
    src.note_retry(0);
    ASSERT_FALSE(src.should_retry(100));
    ASSERT_TRUE(src.should_retry(1000));
    for (int i = 0; i < 20; ++i) src.note_retry(i * 10000);
    ASSERT_TRUE(src.should_retry(200000 + 5001));
}

static void run_tests()
{
    test_reads_complete_sentences();
    test_reassembles_split_writes();
    test_oversized_line_is_discarded();
    test_torn_input_rejected_and_counted();
    test_open_failure_and_backoff();
}

TEST_MAIN
