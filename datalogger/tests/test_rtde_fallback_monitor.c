/*
 * test_rtde_fallback_monitor.c - test harness for the RTDE fallback monitor.
 *
 * Hand-rolled, no framework (the target and build machines get no new
 * dependency).  Exits non-zero if any check fails.  See
 * ../../docs/superpower/plans/plan_rtde_fallback_monitor.md section 3.
 *
 * The tool's single translation unit is included directly, with its main()
 * compiled out, so the tests call the real functions rather than a copy.
 */

#define RTDE_TEST_BUILD 1
#include "../rtde_fallback_monitor.c"

/* ------------------------------------------------------------------ */
/* Minimal check harness                                              */
/* ------------------------------------------------------------------ */

static int g_checks = 0;
static int g_failures = 0;
static const char *g_group = "";

#define GROUP(name) do { g_group = (name); printf("-- %s\n", (name)); } while (0)

#define CHECK(cond)                                                          \
    do {                                                                     \
        g_checks++;                                                          \
        if (!(cond)) {                                                       \
            g_failures++;                                                    \
            printf("   FAIL [%s] line %d: %s\n", g_group, __LINE__, #cond);  \
        }                                                                    \
    } while (0)

#define CHECK_U32(actual, expected)                                          \
    do {                                                                     \
        uint32_t a_ = (uint32_t)(actual), e_ = (uint32_t)(expected);         \
        g_checks++;                                                          \
        if (a_ != e_) {                                                      \
            g_failures++;                                                    \
            printf("   FAIL [%s] line %d: got %lu, want %lu\n",              \
                   g_group, __LINE__,                                        \
                   (unsigned long)a_, (unsigned long)e_);                    \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                    \
    do {                                                                     \
        double a_ = (double)(actual), e_ = (double)(expected);               \
        g_checks++;                                                          \
        if (!(fabs(a_ - e_) <= (tol))) {                                     \
            g_failures++;                                                    \
            printf("   FAIL [%s] line %d: got %.9f, want %.9f\n",            \
                   g_group, __LINE__, a_, e_);                               \
        }                                                                    \
    } while (0)

#define CHECK_STR(actual, expected)                                          \
    do {                                                                     \
        const char *a_ = (actual), *e_ = (expected);                         \
        g_checks++;                                                          \
        if (strcmp(a_, e_) != 0) {                                           \
            g_failures++;                                                    \
            printf("   FAIL [%s] line %d:\n     got  \"%s\"\n"               \
                   "     want \"%s\"\n", g_group, __LINE__, a_, e_);         \
        }                                                                    \
    } while (0)

/* ------------------------------------------------------------------ */
/* Group A - big-endian decode, pinned to known byte sequences         */
/* ------------------------------------------------------------------ */

static void test_read_be_u16(void)
{
    const unsigned char b1[2] = { 0x01, 0x02 };
    const unsigned char b2[2] = { 0x00, 0x00 };
    const unsigned char b3[2] = { 0xFF, 0xFF };
    const unsigned char b4[2] = { 0x00, 0x03 };

    GROUP("read_be_u16");
    CHECK_U32(read_be_u16(b1), 0x0102u);
    CHECK_U32(read_be_u16(b2), 0u);
    CHECK_U32(read_be_u16(b3), 0xFFFFu);
    CHECK_U32(read_be_u16(b4), 3u);
}

static void test_read_be_u32(void)
{
    const unsigned char zero[4] = { 0x00, 0x00, 0x00, 0x00 };
    const unsigned char one[4]  = { 0x00, 0x00, 0x00, 0x01 };
    const unsigned char play[4] = { 0x00, 0x00, 0x00, 0x02 };
    const unsigned char dead[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
    const unsigned char max[4]  = { 0xFF, 0xFF, 0xFF, 0xFF };

    GROUP("read_be_u32");
    CHECK_U32(read_be_u32(zero), 0u);
    CHECK_U32(read_be_u32(one), 1u);
    CHECK_U32(read_be_u32(play), (uint32_t)RT_PLAYING);
    CHECK_U32(read_be_u32(dead), 0xDEADBEEFu);
    CHECK_U32(read_be_u32(max), 0xFFFFFFFFu);
}

static void test_read_be_double(void)
{
    /* IEEE-754 binary64, network (big-endian) byte order. */
    const unsigned char d_zero[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
    const unsigned char d_one[8]  = { 0x3F,0xF0,0x00,0x00,0x00,0x00,0x00,0x00 };
    const unsigned char d_six[8]  = { 0x40,0x18,0x00,0x00,0x00,0x00,0x00,0x00 };
    const unsigned char d_neg[8]  = { 0xC0,0x04,0x00,0x00,0x00,0x00,0x00,0x00 };

    GROUP("read_be_double");
    CHECK_NEAR(read_be_double(d_zero), 0.0, 0.0);
    CHECK_NEAR(read_be_double(d_one), 1.0, 0.0);
    CHECK_NEAR(read_be_double(d_six), 6.0, 0.0);
    CHECK_NEAR(read_be_double(d_neg), -2.5, 0.0);
}

/*
 * The decoders must not assume the field starts on an aligned address:
 * inside an RTDE payload every field sits at an arbitrary offset behind a
 * 3-byte header.  A pointer-cast implementation can pass the aligned cases
 * above and fail here.
 */
static void test_decode_is_alignment_safe(void)
{
    unsigned char buf[16];
    const unsigned char d_one[8] = { 0x3F,0xF0,0x00,0x00,0x00,0x00,0x00,0x00 };
    const unsigned char u_two[4] = { 0x00,0x00,0x00,0x02 };
    size_t off;

    GROUP("decode alignment safety");
    for (off = 1; off <= 7; off++) {
        memset(buf, 0xAA, sizeof(buf));
        memcpy(buf + off, d_one, 8);
        CHECK_NEAR(read_be_double(buf + off), 1.0, 0.0);

        memset(buf, 0xAA, sizeof(buf));
        memcpy(buf + off, u_two, 4);
        CHECK_U32(read_be_u32(buf + off), 2u);
    }
}

/* Round-trip against a locally built big-endian encoder, for arbitrary
 * values that have no hand-checkable hex literal. */
static void be_encode_double(unsigned char *out, double v)
{
    unsigned char tmp[8];
    int i;
    memcpy(tmp, &v, 8);
    for (i = 0; i < 8; i++) {
        out[i] = tmp[7 - i]; /* host here is little-endian x86-64 */
    }
}

static void test_read_be_double_round_trip(void)
{
    static const double vals[] = {
        -6.012345, 0.412345, -0.298765, 0.101234, 1234.56789, -1e-9
    };
    unsigned char enc[8];
    size_t i;

    GROUP("read_be_double round trip");
    for (i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
        be_encode_double(enc, vals[i]);
        CHECK_NEAR(read_be_double(enc), vals[i], 0.0);
    }
}

/* ------------------------------------------------------------------ */
/* Group B - runtime_state transition table                           */
/* ------------------------------------------------------------------ */

/*
 * Rule (plan section 0.4): open on a transition INTO PLAYING from STOPPED,
 * close on any transition INTO STOPPED, and do nothing for the
 * PAUSING/PAUSED/RESUMING excursions that belong to the same run.
 * Every ordered pair is exercised, not only the happy path.
 */
static void test_file_action_opens_a_new_run(void)
{
    GROUP("decide_file_action: open");
    CHECK(decide_file_action(RT_STOPPED, RT_PLAYING) == FILE_ACTION_OPEN);
    /* Tool started while a program is already running: still capture it. */
    CHECK(decide_file_action(RT_UNKNOWN, RT_PLAYING) == FILE_ACTION_OPEN);
}

/*
 * Starting the monitor while the operator has the trial paused must also
 * capture the rest of that trial.  Waiting for a STOPPED->PLAYING edge would
 * silently discard everything until the next trial, which is the worst
 * possible failure for a tool whose whole job is redundancy.
 */
static void test_file_action_opens_when_attaching_to_a_paused_run(void)
{
    GROUP("decide_file_action: attach mid-run");
    CHECK(decide_file_action(RT_UNKNOWN, RT_PAUSING) == FILE_ACTION_OPEN);
    CHECK(decide_file_action(RT_UNKNOWN, RT_PAUSED) == FILE_ACTION_OPEN);
    CHECK(decide_file_action(RT_UNKNOWN, RT_RESUMING) == FILE_ACTION_OPEN);
    /* A run already on its way out is not worth a file. */
    CHECK(decide_file_action(RT_UNKNOWN, RT_STOPPING) == FILE_ACTION_NONE);
}

static void test_file_action_closes_on_stop(void)
{
    GROUP("decide_file_action: close");
    CHECK(decide_file_action(RT_PLAYING, RT_STOPPED) == FILE_ACTION_CLOSE);
    CHECK(decide_file_action(RT_STOPPING, RT_STOPPED) == FILE_ACTION_CLOSE);
    CHECK(decide_file_action(RT_PAUSED, RT_STOPPED) == FILE_ACTION_CLOSE);
    CHECK(decide_file_action(RT_PAUSING, RT_STOPPED) == FILE_ACTION_CLOSE);
    CHECK(decide_file_action(RT_RESUMING, RT_STOPPED) == FILE_ACTION_CLOSE);
}

static void test_file_action_keeps_file_across_pause(void)
{
    GROUP("decide_file_action: pause keeps the same file");
    CHECK(decide_file_action(RT_PLAYING, RT_PAUSING) == FILE_ACTION_NONE);
    CHECK(decide_file_action(RT_PAUSING, RT_PAUSED) == FILE_ACTION_NONE);
    CHECK(decide_file_action(RT_PAUSED, RT_RESUMING) == FILE_ACTION_NONE);
    CHECK(decide_file_action(RT_RESUMING, RT_PLAYING) == FILE_ACTION_NONE);
    /* The pendant can also go straight from PAUSED back to PLAYING. */
    CHECK(decide_file_action(RT_PAUSED, RT_PLAYING) == FILE_ACTION_NONE);
}

static void test_file_action_full_transition_table(void)
{
    uint32_t states[6];
    size_t i, j;

    states[0] = RT_STOPPING;
    states[1] = RT_STOPPED;
    states[2] = RT_PLAYING;
    states[3] = RT_PAUSING;
    states[4] = RT_PAUSED;
    states[5] = RT_RESUMING;

    GROUP("decide_file_action: every ordered pair");
    for (i = 0; i < 6; i++) {
        for (j = 0; j < 6; j++) {
            uint32_t prev = states[i], next = states[j];
            file_action_t want;

            if (next == RT_PLAYING && prev == RT_STOPPED) {
                want = FILE_ACTION_OPEN;
            } else if (next == RT_STOPPED && prev != RT_STOPPED) {
                want = FILE_ACTION_CLOSE;
            } else {
                want = FILE_ACTION_NONE;
            }
            CHECK(decide_file_action(prev, next) == want);
        }
    }
    /* No state change at all is never an action. */
    for (i = 0; i < 6; i++) {
        CHECK(decide_file_action(states[i], states[i]) == FILE_ACTION_NONE);
    }
}

/* ------------------------------------------------------------------ */
/* Group C - decimation to the 20 ms output grid                      */
/* ------------------------------------------------------------------ */

/*
 * The controller streams at its own base rate, which is not confirmed for
 * this PolyScope build (plan section 2), so the cadence is imposed here from
 * each packet's own timestamp.  The rule is grid-based: emit the first
 * packet at or after each 20 ms boundary.  A gap-based rule ("emit when
 * 20 ms have passed since the last emitted packet") would yield 125/3 =
 * 41.7 Hz on a 125 Hz stream instead of the required 50 Hz, which is the
 * regression this group pins down.
 */
static void test_decimation_emits_the_first_sample(void)
{
    double next = 0.0;

    GROUP("decimation: first sample");
    decimate_init(&next, 100.0);
    CHECK(decimate_should_emit(100.0, &next, 0.020) == 1);
}

static void test_decimation_of_a_125hz_stream_averages_50hz(void)
{
    double next = 0.0;
    double t;
    int i, emitted = 0;
    double first_emit = -1.0, last_emit = -1.0;

    GROUP("decimation: 125 Hz in, 50 Hz out");
    decimate_init(&next, 0.0);
    for (i = 0; i < 1250; i++) {          /* 10 s of 125 Hz packets */
        t = i * 0.008;
        if (decimate_should_emit(t, &next, 0.020)) {
            if (first_emit < 0.0) {
                first_emit = t;
            }
            last_emit = t;
            emitted++;
        }
    }
    /* 10 s at 50 Hz is 500 rows, +/- one boundary sample. */
    CHECK(emitted >= 499 && emitted <= 501);
    CHECK_NEAR((emitted - 1) / (last_emit - first_emit), 50.0, 0.2);
}

static void test_decimation_of_a_500hz_stream_averages_50hz(void)
{
    double next = 0.0;
    int i, emitted = 0;

    GROUP("decimation: 500 Hz in, 50 Hz out");
    decimate_init(&next, 0.0);
    for (i = 0; i < 5000; i++) {          /* 10 s of 500 Hz packets */
        if (decimate_should_emit(i * 0.002, &next, 0.020)) {
            emitted++;
        }
    }
    CHECK(emitted >= 499 && emitted <= 501);
}

static void test_decimation_does_not_burst_after_a_stall(void)
{
    double next = 0.0;
    int emitted = 0;
    int i;

    GROUP("decimation: no burst after a stream stall");
    decimate_init(&next, 0.0);
    (void)decimate_should_emit(0.0, &next, 0.020);
    /* One second of silence, then the stream resumes at 125 Hz. */
    for (i = 0; i < 10; i++) {
        if (decimate_should_emit(1.0 + i * 0.008, &next, 0.020)) {
            emitted++;
        }
    }
    /* 10 packets spanning 72 ms must give 4 rows, not 50 catch-up rows. */
    CHECK(emitted >= 3 && emitted <= 5);
}

/* ------------------------------------------------------------------ */
/* Group D - CSV formatting                                           */
/* ------------------------------------------------------------------ */

static void test_csv_row_matches_the_documented_format(void)
{
    char buf[256];
    const double force[3] = { -0.123456, 0.234567, -6.012345 };
    const double pose[3]  = {  0.412345, -0.298765, 0.101234 };

    GROUP("format_csv_row");
    CHECK(format_csv_row(buf, sizeof(buf), 0.0, force, pose) > 0);
    CHECK_STR(buf,
        "0.000,-0.123456,0.234567,-6.012345,0.412345,-0.298765,0.101234\n");
}

static void test_csv_row_time_resolution_is_one_millisecond(void)
{
    char buf[256];
    const double zeros[3] = { 0.0, 0.0, 0.0 };

    GROUP("format_csv_row: time column");
    CHECK(format_csv_row(buf, sizeof(buf), 12.3456, zeros, zeros) > 0);
    CHECK_STR(buf, "12.346,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000\n");
}

static void test_csv_row_refuses_a_short_buffer(void)
{
    char buf[8];
    const double zeros[3] = { 0.0, 0.0, 0.0 };

    GROUP("format_csv_row: short buffer");
    CHECK(format_csv_row(buf, sizeof(buf), 0.0, zeros, zeros) < 0);
}

static void test_csv_header_carries_the_schema_and_provenance(void)
{
    char buf[1024];

    GROUP("format_csv_header");
    CHECK(format_csv_header(buf, sizeof(buf), "192.168.4.38", 30004,
                            "2026-08-14", "10:15:30", 123456.789) > 0);
    CHECK(strstr(buf, "# Robot Model: UR5 CB3\n") != NULL);
    CHECK(strstr(buf, "# PolyScope Version: 3.11.0.82155 (20 August 2019)\n") != NULL);
    CHECK(strstr(buf, "# Data Source: RTDE fallback monitor (192.168.4.14)\n") != NULL);
    CHECK(strstr(buf, "# Robot RTDE Endpoint: 192.168.4.38:30004\n") != NULL);
    CHECK(strstr(buf, "# File Creation Date: 2026-08-14\n") != NULL);
    CHECK(strstr(buf, "# File Creation Time: 10:15:30\n") != NULL);
    CHECK(strstr(buf, "# Target Acquisition Frequency: 50 Hz\n") != NULL);
    /*
     * The plan left the Time column open (section 5), so the file states its
     * own convention and never needs a second document to be read.  It is the
     * robot's own RTDE timestamp, referenced to the first sample of the file,
     * so it starts at 0.000 exactly like the on-robot path's tick time and the
     * two CSVs line up without any clock conversion.
     */
    CHECK(strstr(buf, "# Time Column: RTDE timestamp field, relative to the "
                      "first sample of this file (s)\n") != NULL);
    /* The absolute controller clock is kept so nothing is lost by the offset. */
    CHECK(strstr(buf, "# RTDE Timestamp At First Sample: 123456.789000 s "
                      "(controller uptime)\n") != NULL);
    /* Schema line, last, exactly as the on-robot path writes it. */
    CHECK(strstr(buf,
        "Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ\n") != NULL);
}

/*
 * A malformed address is an operator typo, not a transient network fault, so
 * it has to be rejected before the reconnect loop starts - otherwise the tool
 * retries a name that can never resolve, every 2 s, forever.  Legacy inet_addr
 * forms ("192.168.4", octal, hex) are refused too: on this isolated VLAN the
 * only correct input is four dotted decimal octets, and silently accepting
 * "192.168.4" as 192.168.0.4 would connect to the wrong machine.
 */
static void test_ipv4_validation(void)
{
    GROUP("is_valid_ipv4");
    CHECK(is_valid_ipv4("192.168.4.38") == 1);
    CHECK(is_valid_ipv4("127.0.0.1") == 1);
    CHECK(is_valid_ipv4("0.0.0.0") == 1);
    CHECK(is_valid_ipv4("255.255.255.255") == 1);

    CHECK(is_valid_ipv4("192.168.4") == 0);
    CHECK(is_valid_ipv4("192.168.4.38.1") == 0);
    CHECK(is_valid_ipv4("192.168.4.256") == 0);
    CHECK(is_valid_ipv4("192.168.4.") == 0);
    CHECK(is_valid_ipv4(".168.4.38") == 0);
    CHECK(is_valid_ipv4("192.168..38") == 0);
    CHECK(is_valid_ipv4("robot.local") == 0);
    CHECK(is_valid_ipv4("") == 0);
}

static void test_csv_filename_is_stamped_and_prefixed(void)
{
    char buf[512];

    GROUP("format_csv_filename");
    CHECK(format_csv_filename(buf, sizeof(buf), ".", "20260814_101530") > 0);
    CHECK_STR(buf, ".\\ACQ_rtde_20260814_101530.csv");

    CHECK(format_csv_filename(buf, sizeof(buf), "C:\\data\\", "20260814_101530") > 0);
    CHECK_STR(buf, "C:\\data\\ACQ_rtde_20260814_101530.csv");
}

/* ================================================================== */
/* Group E - integration against a local fake RTDE server             */
/* ================================================================== */

/*
 * The fake server speaks the documented RTDE handshake on 127.0.0.1 and then
 * replays a scripted runtime_state sequence, so the whole socket path - the
 * handshake, the packet framing, the file boundaries, the decimation and the
 * disconnect handling - is exercised with no robot and no network.
 */

typedef struct {
    SOCKET listener;
    int reject_v2;          /* answer "not accepted" to a version-2 request */
    int emit_not_found;     /* answer the recipe with a NOT_FOUND field type */
    const uint32_t *states; /* one runtime_state per data packet */
    int n_states;
    double t_start;         /* controller timestamp of the first packet */
    double dt;              /* timestamp step between packets */
    int abort_mid_stream;   /* drop the connection abruptly, mid-run */
    int inject_text;        /* interleave RTDE_TEXT_MESSAGE packets */
} fake_server_cfg_t;

static int fake_send_packet(SOCKET s, unsigned char type,
                            const unsigned char *payload, size_t n)
{
    unsigned char pkt[RTDE_MAX_PACKET];
    size_t total = RTDE_HEADER_SIZE + n;

    write_be_u16(pkt, (uint16_t)total);
    pkt[2] = type;
    if (n > 0) {
        memcpy(pkt + RTDE_HEADER_SIZE, payload, n);
    }
    return send(s, (const char *)pkt, (int)total, 0) == (int)total ? 0 : -1;
}

static int fake_recv_packet(SOCKET s, unsigned char *type,
                            unsigned char *payload, size_t *n)
{
    unsigned char hdr[RTDE_HEADER_SIZE];
    int got = 0;
    uint16_t size;

    while (got < RTDE_HEADER_SIZE) {
        int r = recv(s, (char *)hdr + got, RTDE_HEADER_SIZE - got, 0);
        if (r <= 0) {
            return -1;
        }
        got += r;
    }
    size = read_be_u16(hdr);
    *type = hdr[2];
    *n = (size_t)size - RTDE_HEADER_SIZE;
    got = 0;
    while ((size_t)got < *n) {
        int r = recv(s, (char *)payload + got, (int)(*n - (size_t)got), 0);
        if (r <= 0) {
            return -1;
        }
        got += r;
    }
    return 0;
}

static void fake_build_payload(unsigned char *out, double ts, uint32_t state,
                               double seed)
{
    int i;

    write_be_double(out + FIELD_OFF_TIMESTAMP, ts);
    for (i = 0; i < 6; i++) {
        write_be_double(out + FIELD_OFF_TCP_POSE + 8 * i, seed + 0.1 * i);
    }
    for (i = 0; i < 6; i++) {
        write_be_double(out + FIELD_OFF_TCP_FORCE + 8 * i, -seed - 0.5 * i);
    }
    out[FIELD_OFF_RUNTIME_STATE + 0] = (unsigned char)((state >> 24) & 0xFF);
    out[FIELD_OFF_RUNTIME_STATE + 1] = (unsigned char)((state >> 16) & 0xFF);
    out[FIELD_OFF_RUNTIME_STATE + 2] = (unsigned char)((state >> 8) & 0xFF);
    out[FIELD_OFF_RUNTIME_STATE + 3] = (unsigned char)(state & 0xFF);
}

/*
 * A real controller emits RTDE_TEXT_MESSAGE packets whenever it feels like
 * it, including between a request and its reply.  The fake server can do the
 * same so the client is forced to skip them instead of mistaking one for the
 * answer it was waiting for.
 */
static void maybe_text(SOCKET c, const fake_server_cfg_t *cfg)
{
    static const unsigned char msg[] = { 5, 'h', 'e', 'l', 'l', 'o', 1 };

    if (cfg->inject_text) {
        fake_send_packet(c, RTDE_TEXT_MESSAGE, msg, sizeof(msg));
    }
}

static DWORD WINAPI fake_server_thread(LPVOID arg)
{
    fake_server_cfg_t *cfg = (fake_server_cfg_t *)arg;
    unsigned char payload[RTDE_MAX_PACKET];
    unsigned char reply[RTDE_MAX_PACKET];
    unsigned char type;
    size_t n;
    SOCKET c;
    int negotiated = 1;
    uint8_t recipe_id = 1;
    int i;

    c = accept(cfg->listener, NULL, NULL);
    if (c == INVALID_SOCKET) {
        return 1;
    }

    /* 1. protocol version */
    if (fake_recv_packet(c, &type, payload, &n) != 0 ||
        type != RTDE_REQUEST_PROTOCOL_VERSION) {
        closesocket(c);
        return 1;
    }
    {
        uint16_t want = read_be_u16(payload);
        int accepted = (want == 2 && cfg->reject_v2) ? 0 : 1;
        if (accepted) {
            negotiated = (int)want;
        }
        reply[0] = (unsigned char)accepted;
        maybe_text(c, cfg);
        fake_send_packet(c, RTDE_REQUEST_PROTOCOL_VERSION, reply, 1);
        if (!accepted) {
            /* The client is expected to come back asking for version 1. */
            if (fake_recv_packet(c, &type, payload, &n) != 0 ||
                type != RTDE_REQUEST_PROTOCOL_VERSION) {
                closesocket(c);
                return 1;
            }
            negotiated = (int)read_be_u16(payload);
            reply[0] = 1;
            fake_send_packet(c, RTDE_REQUEST_PROTOCOL_VERSION, reply, 1);
        }
    }

    /* 2. output recipe */
    if (fake_recv_packet(c, &type, payload, &n) != 0 ||
        type != RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS) {
        closesocket(c);
        return 1;
    }
    {
        const char *types = cfg->emit_not_found
            ? "DOUBLE,VECTOR6D,VECTOR6D,NOT_FOUND"
            : "DOUBLE,VECTOR6D,VECTOR6D,UINT32";
        size_t len = strlen(types);
        size_t off = 0;

        if (negotiated >= 2) {
            reply[0] = recipe_id;
            off = 1;
        }
        memcpy(reply + off, types, len);
        maybe_text(c, cfg);
        fake_send_packet(c, RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, reply, off + len);
        if (cfg->emit_not_found) {
            /* The client must abort here rather than log garbage. */
            closesocket(c);
            return 0;
        }
    }

    /* 3. start */
    if (fake_recv_packet(c, &type, payload, &n) != 0 ||
        type != RTDE_CONTROL_PACKAGE_START) {
        closesocket(c);
        return 1;
    }
    reply[0] = 1;
    maybe_text(c, cfg);
    fake_send_packet(c, RTDE_CONTROL_PACKAGE_START, reply, 1);

    /* 4. stream */
    for (i = 0; i < cfg->n_states; i++) {
        unsigned char body[1 + RTDE_PAYLOAD_SIZE];
        size_t off = 0;

        if (negotiated >= 2) {
            body[0] = recipe_id;
            off = 1;
        }
        fake_build_payload(body + off, cfg->t_start + i * cfg->dt,
                           cfg->states[i], 0.4 + 0.001 * i);
        if (fake_send_packet(c, RTDE_DATA_PACKAGE, body, off + RTDE_PAYLOAD_SIZE) != 0) {
            break;
        }
        if (cfg->abort_mid_stream && i == cfg->n_states / 2) {
            /* Controller reboot / cable pull: reset the connection so the
             * client sees an error, not an orderly end of stream. */
            struct linger lg;
            lg.l_onoff = 1;
            lg.l_linger = 0;
            setsockopt(c, SOL_SOCKET, SO_LINGER, (const char *)&lg, sizeof(lg));
            closesocket(c);
            return 0;
        }
    }
    closesocket(c);
    return 0;
}

/* -- test-side filesystem helpers ---------------------------------- */

static void rmdir_recursive(const char *dir)
{
    char pattern[MAX_PATH];
    char path[MAX_PATH];
    WIN32_FIND_DATAA fd;
    HANDLE h;

    snprintf(pattern, sizeof(pattern), "%s\\*", dir);
    h = FindFirstFileA(pattern, &fd);
    if (h != INVALID_HANDLE_VALUE) {
        do {
            if (strcmp(fd.cFileName, ".") == 0 || strcmp(fd.cFileName, "..") == 0) {
                continue;
            }
            snprintf(path, sizeof(path), "%s\\%s", dir, fd.cFileName);
            DeleteFileA(path);
        } while (FindNextFileA(h, &fd));
        FindClose(h);
    }
    RemoveDirectoryA(dir);
}

static void fresh_dir(const char *dir)
{
    rmdir_recursive(dir);
    CreateDirectoryA(dir, NULL);
}

static int list_csv(const char *dir, char names[8][MAX_PATH])
{
    char pattern[MAX_PATH];
    WIN32_FIND_DATAA fd;
    HANDLE h;
    int count = 0;

    snprintf(pattern, sizeof(pattern), "%s\\ACQ_rtde_*.csv", dir);
    h = FindFirstFileA(pattern, &fd);
    if (h == INVALID_HANDLE_VALUE) {
        return 0;
    }
    do {
        if (count < 8) {
            snprintf(names[count], MAX_PATH, "%s\\%s", dir, fd.cFileName);
            count++;
        }
    } while (FindNextFileA(h, &fd));
    FindClose(h);
    return count;
}

static char *read_text(const char *path, size_t *out_len)
{
    FILE *f = fopen(path, "rb");
    char *buf;
    long len;

    if (!f) {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = (char *)malloc((size_t)len + 1);
    if (!buf) {
        fclose(f);
        return NULL;
    }
    if (fread(buf, 1, (size_t)len, f) != (size_t)len) {
        free(buf);
        fclose(f);
        return NULL;
    }
    buf[len] = '\0';
    fclose(f);
    if (out_len) {
        *out_len = (size_t)len;
    }
    return buf;
}

/* Number of data rows, i.e. lines that are neither comments nor the schema. */
static int count_data_rows(const char *text)
{
    const char *p = text;
    int rows = 0;

    while (*p) {
        const char *eol = strchr(p, '\n');
        if (!eol) {
            break;
        }
        if (*p != '#' && strncmp(p, "Time,", 5) != 0 && eol > p) {
            rows++;
        }
        p = eol + 1;
    }
    return rows;
}

static const char *first_data_row(const char *text)
{
    const char *p = strstr(text, CSV_SCHEMA_LINE);
    return p ? p + strlen(CSV_SCHEMA_LINE) : NULL;
}

/* -- the integration scenarios ------------------------------------- */

static int run_against_fake(fake_server_cfg_t *cfg, const char *out_dir)
{
    struct sockaddr_in addr;
    int addrlen = sizeof(addr);
    HANDLE th;
    int port, rc;

    cfg->listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;                       /* let Windows pick a free port */
    bind(cfg->listener, (struct sockaddr *)&addr, sizeof(addr));
    listen(cfg->listener, 1);
    getsockname(cfg->listener, (struct sockaddr *)&addr, &addrlen);
    port = ntohs(addr.sin_port);

    th = CreateThread(NULL, 0, fake_server_thread, cfg, 0, NULL);
    rc = monitor_run_once("127.0.0.1", port, out_dir);
    WaitForSingleObject(th, 5000);
    CloseHandle(th);
    closesocket(cfg->listener);
    return rc;
}

static void test_one_run_produces_one_csv(void)
{
    static uint32_t states[202];
    fake_server_cfg_t cfg;
    char names[8][MAX_PATH];
    char *text;
    int i, n;
    const char *dir = "_tmp_e1";

    GROUP("integration: one run, one CSV");
    states[0] = RT_STOPPED;
    for (i = 1; i <= 200; i++) {
        states[i] = RT_PLAYING;      /* 200 packets at 8 ms = 1.6 s */
    }
    states[201] = RT_STOPPED;

    memset(&cfg, 0, sizeof(cfg));
    cfg.states = states;
    cfg.n_states = 202;
    cfg.t_start = 123456.789;
    cfg.dt = 0.008;

    fresh_dir(dir);
    CHECK(run_against_fake(&cfg, dir) == MON_OK);

    n = list_csv(dir, names);
    CHECK_U32(n, 1u);
    if (n != 1) {
        return;
    }
    text = read_text(names[0], NULL);
    CHECK(text != NULL);
    if (!text) {
        return;
    }

    CHECK(strstr(text, "# Robot Model: UR5 CB3\n") != NULL);
    CHECK(strstr(text, CSV_SCHEMA_LINE) != NULL);
    /*
     * The absolute controller clock of the file's first sample is recorded.
     * That is the first PLAYING packet (t_start + one step), not the STOPPED
     * packet that preceded it - the stop belongs to no run.
     */
    CHECK(strstr(text, "# RTDE Timestamp At First Sample: 123456.797000") != NULL);
    /* Time is referenced to that sample, so the first row starts at zero. */
    CHECK(first_data_row(text) != NULL);
    if (first_data_row(text)) {
        CHECK(strncmp(first_data_row(text), "0.000,", 6) == 0);
    }
    /* 1.6 s decimated to the 20 ms grid is 80 rows, give or take a boundary. */
    n = count_data_rows(text);
    CHECK(n >= 79 && n <= 82);

    free(text);
    rmdir_recursive(dir);
}

static void test_pause_does_not_split_the_file(void)
{
    static uint32_t states[156];
    fake_server_cfg_t cfg;
    char names[8][MAX_PATH];
    int i, k = 0;
    const char *dir = "_tmp_e2";

    GROUP("integration: pause keeps one file");
    states[k++] = RT_STOPPED;
    for (i = 0; i < 50; i++) { states[k++] = RT_PLAYING; }
    states[k++] = RT_PAUSING;
    for (i = 0; i < 50; i++) { states[k++] = RT_PAUSED; }
    states[k++] = RT_RESUMING;
    for (i = 0; i < 50; i++) { states[k++] = RT_PLAYING; }
    states[k++] = RT_STOPPED;

    memset(&cfg, 0, sizeof(cfg));
    cfg.states = states;
    cfg.n_states = k;
    cfg.t_start = 10.0;
    cfg.dt = 0.008;

    fresh_dir(dir);
    CHECK(run_against_fake(&cfg, dir) == MON_OK);
    CHECK_U32(list_csv(dir, names), 1u);
    rmdir_recursive(dir);
}

static void test_two_runs_produce_two_distinct_files(void)
{
    static uint32_t states[103];
    fake_server_cfg_t cfg;
    char names[8][MAX_PATH];
    int i, k = 0, n;
    const char *dir = "_tmp_e3";

    GROUP("integration: two runs, two files");
    states[k++] = RT_STOPPED;
    for (i = 0; i < 50; i++) { states[k++] = RT_PLAYING; }
    states[k++] = RT_STOPPED;
    for (i = 0; i < 50; i++) { states[k++] = RT_PLAYING; }
    states[k++] = RT_STOPPED;

    memset(&cfg, 0, sizeof(cfg));
    cfg.states = states;
    cfg.n_states = k;
    cfg.t_start = 500.0;
    cfg.dt = 0.008;

    fresh_dir(dir);
    CHECK(run_against_fake(&cfg, dir) == MON_OK);
    /* Both runs land inside the same wall-clock second, so this also pins
     * down that a same-second name collision never overwrites the first
     * trial's data. */
    n = list_csv(dir, names);
    CHECK_U32(n, 2u);
    if (n == 2) {
        CHECK(strcmp(names[0], names[1]) != 0);
    }
    rmdir_recursive(dir);
}

static void test_midstream_disconnect_preserves_the_partial_file(void)
{
    static uint32_t states[201];
    fake_server_cfg_t cfg;
    char names[8][MAX_PATH];
    char *text;
    size_t len = 0;
    int i, n;
    const char *dir = "_tmp_e4";

    GROUP("integration: mid-stream disconnect keeps the partial file");
    states[0] = RT_STOPPED;
    for (i = 1; i < 201; i++) {
        states[i] = RT_PLAYING;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.states = states;
    cfg.n_states = 201;
    cfg.t_start = 7.0;
    cfg.dt = 0.008;
    cfg.abort_mid_stream = 1;

    fresh_dir(dir);
    /* A reset connection is an error, but never a crash and never data loss. */
    CHECK(run_against_fake(&cfg, dir) == MON_ERR_STREAM);
    n = list_csv(dir, names);
    CHECK_U32(n, 1u);
    if (n != 1) {
        return;
    }
    text = read_text(names[0], &len);
    CHECK(text != NULL);
    if (!text) {
        return;
    }
    CHECK(count_data_rows(text) > 0);
    /* Whatever was written is a complete, parsable file: last byte ends a row. */
    CHECK(len > 0 && text[len - 1] == '\n');
    free(text);
    rmdir_recursive(dir);
}

static void test_unsupported_field_aborts_before_logging(void)
{
    static uint32_t states[10];
    fake_server_cfg_t cfg;
    char names[8][MAX_PATH];
    int i;
    const char *dir = "_tmp_e5";

    GROUP("integration: NOT_FOUND field aborts the handshake");
    for (i = 0; i < 10; i++) {
        states[i] = RT_PLAYING;
    }
    memset(&cfg, 0, sizeof(cfg));
    cfg.states = states;
    cfg.n_states = 10;
    cfg.t_start = 1.0;
    cfg.dt = 0.008;
    cfg.emit_not_found = 1;

    fresh_dir(dir);
    /* A field this build does not support must be a clean, loud refusal, not
     * a stream of mis-parsed numbers. */
    CHECK(run_against_fake(&cfg, dir) == MON_ERR_HANDSHAKE);
    CHECK_U32(list_csv(dir, names), 0u);
    rmdir_recursive(dir);
}

static void test_falls_back_to_protocol_version_1(void)
{
    static uint32_t states[102];
    fake_server_cfg_t cfg;
    char names[8][MAX_PATH];
    int i;
    const char *dir = "_tmp_e6";

    GROUP("integration: protocol version 1 fallback");
    states[0] = RT_STOPPED;
    for (i = 1; i <= 100; i++) {
        states[i] = RT_PLAYING;
    }
    states[101] = RT_STOPPED;

    memset(&cfg, 0, sizeof(cfg));
    cfg.states = states;
    cfg.n_states = 102;
    cfg.t_start = 42.0;
    cfg.dt = 0.008;
    cfg.reject_v2 = 1;

    fresh_dir(dir);
    /* Version 1 carries no recipe id in the data packages and no frequency
     * field in the setup, so the framing differs; the output must not. */
    CHECK(run_against_fake(&cfg, dir) == MON_OK);
    CHECK_U32(list_csv(dir, names), 1u);
    rmdir_recursive(dir);
}

static void test_text_messages_do_not_break_the_handshake(void)
{
    static uint32_t states[102];
    fake_server_cfg_t cfg;
    char names[8][MAX_PATH];
    int i;
    const char *dir = "_tmp_e7";

    GROUP("integration: interleaved controller text messages");
    states[0] = RT_STOPPED;
    for (i = 1; i <= 100; i++) {
        states[i] = RT_PLAYING;
    }
    states[101] = RT_STOPPED;

    memset(&cfg, 0, sizeof(cfg));
    cfg.states = states;
    cfg.n_states = 102;
    cfg.t_start = 3.0;
    cfg.dt = 0.008;
    cfg.inject_text = 1;

    fresh_dir(dir);
    /* A text message arriving between a request and its reply must be
     * skipped, not mistaken for the reply and reported as a failed
     * handshake. */
    CHECK(run_against_fake(&cfg, dir) == MON_OK);
    CHECK_U32(list_csv(dir, names), 1u);
    rmdir_recursive(dir);
}

static void test_attaching_to_a_running_program_logs_it(void)
{
    static uint32_t states[101];
    fake_server_cfg_t cfg;
    char names[8][MAX_PATH];
    int i;
    const char *dir = "_tmp_e8";

    GROUP("integration: monitor started mid-trial");
    /* No STOPPED->PLAYING edge anywhere: the stream is already running when
     * the monitor attaches, exactly as when an operator starts the tool late. */
    for (i = 0; i < 101; i++) {
        states[i] = RT_PLAYING;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.states = states;
    cfg.n_states = 101;
    cfg.t_start = 900.0;
    cfg.dt = 0.008;

    fresh_dir(dir);
    CHECK(run_against_fake(&cfg, dir) == MON_OK);
    CHECK_U32(list_csv(dir, names), 1u);
    rmdir_recursive(dir);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    WSADATA wsa;

    printf("== rtde_fallback_monitor unit tests ==\n");

    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("WSAStartup failed\n");
        return 1;
    }

    test_read_be_u16();
    test_read_be_u32();
    test_read_be_double();
    test_decode_is_alignment_safe();
    test_read_be_double_round_trip();

    test_file_action_opens_a_new_run();
    test_file_action_opens_when_attaching_to_a_paused_run();
    test_file_action_closes_on_stop();
    test_file_action_keeps_file_across_pause();
    test_file_action_full_transition_table();

    test_decimation_emits_the_first_sample();
    test_decimation_of_a_125hz_stream_averages_50hz();
    test_decimation_of_a_500hz_stream_averages_50hz();
    test_decimation_does_not_burst_after_a_stall();

    test_csv_row_matches_the_documented_format();
    test_csv_row_time_resolution_is_one_millisecond();
    test_csv_row_refuses_a_short_buffer();
    test_csv_header_carries_the_schema_and_provenance();
    test_csv_filename_is_stamped_and_prefixed();
    test_ipv4_validation();

    test_one_run_produces_one_csv();
    test_pause_does_not_split_the_file();
    test_two_runs_produce_two_distinct_files();
    test_midstream_disconnect_preserves_the_partial_file();
    test_unsupported_field_aborts_before_logging();
    test_falls_back_to_protocol_version_1();
    test_text_messages_do_not_break_the_handshake();
    test_attaching_to_a_running_program_logs_it();

    WSACleanup();

    printf("\n%d checks, %d failure(s)\n", g_checks, g_failures);
    if (g_failures != 0) {
        printf("RESULT: FAIL\n");
        return 1;
    }
    printf("RESULT: PASS\n");
    return 0;
}
