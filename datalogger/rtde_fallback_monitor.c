/*
 * rtde_fallback_monitor.c - passive RTDE recorder for a UR5 CB3.
 *
 * Redundant, lab-computer-side data path for the ISO/COLIPA cosmetic-spread
 * protocol.  Connects out to the robot's Real-Time Data Exchange port, reads
 * TCP force, TCP pose and the program runtime_state, and writes one CSV per
 * robot program run.  It never sends a motion command, a register write, or
 * anything else that could affect the running program: the RTDE input path is
 * not used at all, so the tool is safe to leave connected indefinitely and
 * safe to run beside the independent on-robot logger.
 *
 * Design and rationale: ../docs/superpower/plans/plan_rtde_fallback_monitor.md
 * CSV schema shared with the on-robot path: ../docs/superpower/plans/plan_acq_datalogger.md
 *
 * Build:  gcc -O2 -static -Wall -Wextra -o rtde_fallback_monitor.exe \
 *              rtde_fallback_monitor.c -lws2_32
 * Usage:  rtde_fallback_monitor.exe <robot-ip> <rtde-port> <out-dir>
 */

#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <errno.h>

/* ------------------------------------------------------------------ */
/* Protocol and output constants                                       */
/* ------------------------------------------------------------------ */

/* RTDE package types (one byte, after the 2-byte big-endian size). */
#define RTDE_REQUEST_PROTOCOL_VERSION      86  /* 'V' */
#define RTDE_TEXT_MESSAGE                  77  /* 'M' */
#define RTDE_DATA_PACKAGE                  85  /* 'U' */
#define RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS 79  /* 'O' */
#define RTDE_CONTROL_PACKAGE_START         83  /* 'S' */
#define RTDE_CONTROL_PACKAGE_PAUSE         80  /* 'P' */

#define RTDE_HEADER_SIZE 3
#define RTDE_MAX_PACKET  4096

/*
 * Output recipe.  runtime_state rides along in the same stream as the
 * measurements, which is what lets the tool find its own file boundaries
 * without polling the Dashboard Server on a second connection.
 */
#define RTDE_OUTPUT_RECIPE \
    "timestamp,actual_TCP_pose,actual_TCP_force,runtime_state"

/* Byte layout of one data payload, given that recipe. */
#define FIELD_OFF_TIMESTAMP     0
#define FIELD_OFF_TCP_POSE      8    /* VECTOR6D */
#define FIELD_OFF_TCP_FORCE     56   /* VECTOR6D */
#define FIELD_OFF_RUNTIME_STATE 104  /* UINT32   */
#define RTDE_PAYLOAD_SIZE       108

/* runtime_state enumeration (RTDE UINT32 field). */
#define RT_STOPPING 0u
#define RT_STOPPED  1u
#define RT_PLAYING  2u
#define RT_PAUSING  3u
#define RT_PAUSED   4u
#define RT_RESUMING 5u
/* Not a protocol value: "no packet seen yet", so a tool started mid-run
 * still opens a file instead of waiting for the next program start. */
#define RT_UNKNOWN  0xFFFFFFFFu

#define OUTPUT_GRID_S     0.020   /* 50 Hz target cadence */
#define TARGET_HZ_LABEL   "50 Hz"
#define CSV_SCHEMA_LINE   "Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ\n"
#define CSV_FILE_PREFIX   "ACQ_rtde_"

/*
 * Requested output frequency for RTDE protocol version 2, which requires the
 * field.  125 Hz is the CB3 control-loop rate and the documented maximum a
 * CB3 accepts; an e-Series (500 Hz base) accepts it too.  The exact base rate
 * of this PolyScope build is unconfirmed, so nothing downstream depends on
 * this number: the 50 Hz cadence is imposed from each packet's own timestamp
 * (see decimate_should_emit), and protocol version 1 - which has no frequency
 * field at all - is the fallback.
 */
#define RTDE_REQUESTED_HZ 125.0

typedef enum {
    FILE_ACTION_NONE  = 0,
    FILE_ACTION_OPEN  = 1,
    FILE_ACTION_CLOSE = 2
} file_action_t;

/* ------------------------------------------------------------------ */
/* Big-endian decode                                                   */
/* ------------------------------------------------------------------ */

/*
 * RTDE is big-endian; x86-64 is little-endian.  Every multi-byte field in the
 * stream goes through these three functions and never through an inline
 * pointer cast: a cast would be a strict-aliasing and alignment violation
 * (fields sit at arbitrary offsets behind a 3-byte header), and an endianness
 * slip produces a plausible wrong number rather than an error.  One place to
 * get right, one place the tests pin down with known byte sequences.
 */

static uint16_t read_be_u16(const unsigned char *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static uint32_t read_be_u32(const unsigned char *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

static double read_be_double(const unsigned char *p)
{
    unsigned char host[8];
    double out;
    int i;

    for (i = 0; i < 8; i++) {
        host[i] = p[7 - i];
    }
    memcpy(&out, host, sizeof(out));
    return out;
}

static void write_be_u16(unsigned char *p, uint16_t v)
{
    p[0] = (unsigned char)((v >> 8) & 0xFF);
    p[1] = (unsigned char)(v & 0xFF);
}

static void write_be_double(unsigned char *p, double v)
{
    unsigned char host[8];
    int i;

    memcpy(host, &v, sizeof(host));
    for (i = 0; i < 8; i++) {
        p[i] = host[7 - i];
    }
}

/* ------------------------------------------------------------------ */
/* File-boundary decision                                              */
/* ------------------------------------------------------------------ */

/*
 * Open a new CSV on a transition into PLAYING from STOPPED (a genuine new
 * run), close on any transition into STOPPED, and do nothing for the
 * PAUSING / PAUSED / RESUMING excursions, which belong to the same trial: an
 * operator pausing mid-trial must not split the file.
 */
static file_action_t decide_file_action(uint32_t prev, uint32_t next)
{
    int next_is_active = (next == RT_PLAYING  || next == RT_PAUSING ||
                          next == RT_PAUSED   || next == RT_RESUMING);

    if (next == prev) {
        return FILE_ACTION_NONE;
    }
    /*
     * First packet after connecting.  Any state that still belongs to a live
     * trial - paused included - opens a file, so starting the monitor late
     * captures the rest of that trial instead of discarding it while waiting
     * for a STOPPED->PLAYING edge that will not come until the next one.
     */
    if (prev == RT_UNKNOWN) {
        return next_is_active ? FILE_ACTION_OPEN : FILE_ACTION_NONE;
    }
    if (next == RT_PLAYING && prev == RT_STOPPED) {
        return FILE_ACTION_OPEN;
    }
    if (next == RT_STOPPED && prev != RT_STOPPED) {
        return FILE_ACTION_CLOSE;
    }
    return FILE_ACTION_NONE;
}

/* ------------------------------------------------------------------ */
/* Decimation to the output grid                                       */
/* ------------------------------------------------------------------ */

static void decimate_init(double *next_emit, double first_ts)
{
    *next_emit = first_ts;
}

/*
 * Emit the first packet at or after each grid boundary, then advance the
 * boundary past the packet just taken.  Grid-based, not gap-based: "20 ms
 * since the last row" would give 125/3 = 41.7 Hz on a 125 Hz stream, whereas
 * anchoring to the grid averages exactly 50 Hz whatever the base rate is.
 * The catch-up loop also stops a stalled stream from emitting a burst of
 * back-dated rows when it resumes.
 */
static int decimate_should_emit(double ts, double *next_emit, double grid_s)
{
    const double eps = 1e-9;

    if (ts + eps < *next_emit) {
        return 0;
    }
    do {
        *next_emit += grid_s;
    } while (*next_emit <= ts + eps);
    return 1;
}

/* ------------------------------------------------------------------ */
/* CSV formatting                                                      */
/* ------------------------------------------------------------------ */

static int format_csv_row(char *buf, size_t buflen, double t_rel,
                          const double force3[3], const double pose3[3])
{
    int n = snprintf(buf, buflen,
                     "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                     t_rel,
                     force3[0], force3[1], force3[2],
                     pose3[0], pose3[1], pose3[2]);

    if (n < 0 || (size_t)n >= buflen) {
        return -1;
    }
    return n;
}

/*
 * The header is self-describing on purpose: the Time convention is stated in
 * the file, so a CSV never needs a second document to be interpreted, and the
 * absolute controller clock of the first sample is kept so referencing Time
 * to that sample loses nothing.
 */
static int format_csv_header(char *buf, size_t buflen, const char *robot_ip,
                             int robot_port, const char *date_str,
                             const char *time_str, double rtde_t0)
{
    int n = snprintf(buf, buflen,
        "# Robot Model: UR5 CB3\n"
        "# PolyScope Version: 3.11.0.82155 (20 August 2019)\n"
        "# Data Source: RTDE fallback monitor (192.168.4.14)\n"
        "# Robot RTDE Endpoint: %s:%d\n"
        "# File Creation Date: %s\n"
        "# File Creation Time: %s\n"
        "# Target Acquisition Frequency: " TARGET_HZ_LABEL "\n"
        "# Time Column: RTDE timestamp field, relative to the first sample"
        " of this file (s)\n"
        "# RTDE Timestamp At First Sample: %.6f s (controller uptime)\n"
        CSV_SCHEMA_LINE,
        robot_ip, robot_port, date_str, time_str, rtde_t0);

    if (n < 0 || (size_t)n >= buflen) {
        return -1;
    }
    return n;
}

/* ------------------------------------------------------------------ */
/* Address validation                                                  */
/* ------------------------------------------------------------------ */

/*
 * Strict four-octet dotted decimal.  inet_addr() is deliberately not used as
 * the validator: it also accepts the legacy "a.b.c", octal and hex forms, so
 * a mistyped "192.168.4" would be silently read as 192.168.0.4 and the tool
 * would connect to whatever sits at that address on the VLAN.
 */
static int is_valid_ipv4(const char *s)
{
    int octets = 0;

    for (;;) {
        int value = 0, digits = 0;

        while (*s >= '0' && *s <= '9') {
            value = value * 10 + (*s - '0');
            if (++digits > 3 || value > 255) {
                return 0;
            }
            s++;
        }
        if (digits == 0) {
            return 0;
        }
        octets++;
        if (*s == '.') {
            s++;
            continue;
        }
        break;
    }
    return (*s == '\0' && octets == 4) ? 1 : 0;
}

static int format_csv_filename(char *buf, size_t buflen, const char *out_dir,
                               const char *stamp)
{
    size_t len = strlen(out_dir);
    int needs_sep = (len > 0 &&
                     out_dir[len - 1] != '\\' && out_dir[len - 1] != '/');
    int n = snprintf(buf, buflen, "%s%s" CSV_FILE_PREFIX "%s.csv",
                     out_dir, needs_sep ? "\\" : "", stamp);

    if (n < 0 || (size_t)n >= buflen) {
        return -1;
    }
    return n;
}

/* ------------------------------------------------------------------ */
/* CSV writer                                                          */
/* ------------------------------------------------------------------ */

typedef struct {
    FILE *fp;
    char path[MAX_PATH];
    double t0;          /* controller timestamp of this file's first sample */
    double next_emit;   /* next 20 ms grid boundary                         */
    long rows;
} csv_writer_t;

static int file_exists(const char *path)
{
    return GetFileAttributesA(path) != INVALID_FILE_ATTRIBUTES;
}

/*
 * Open the CSV for one robot program run.  The name carries the lab
 * computer's wall clock; a same-second collision (two short trials, or a
 * restart) gets a _1, _2 ... suffix rather than overwriting a trial that has
 * already been recorded.
 */
static int csv_open(csv_writer_t *w, const char *out_dir, const char *robot_ip,
                    int robot_port, double rtde_ts)
{
    char header[1024];
    char stamp[64];
    char date_str[32];
    char time_str[32];
    char base[MAX_PATH];
    time_t now = time(NULL);
    struct tm *lt = localtime(&now);
    int suffix;

    strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", lt);
    strftime(date_str, sizeof(date_str), "%Y-%m-%d", lt);
    strftime(time_str, sizeof(time_str), "%H:%M:%S", lt);

    if (format_csv_filename(w->path, sizeof(w->path), out_dir, stamp) < 0) {
        fprintf(stderr, "[RTDE] output path too long for directory '%s'\n", out_dir);
        return -1;
    }
    for (suffix = 1; file_exists(w->path) && suffix < 100; suffix++) {
        snprintf(base, sizeof(base), "%s_%d", stamp, suffix);
        if (format_csv_filename(w->path, sizeof(w->path), out_dir, base) < 0) {
            return -1;
        }
    }

    w->fp = fopen(w->path, "wb");
    if (!w->fp) {
        fprintf(stderr, "[RTDE] cannot create '%s': %s\n",
                w->path, strerror(errno));
        return -1;
    }
    if (format_csv_header(header, sizeof(header), robot_ip, robot_port,
                          date_str, time_str, rtde_ts) < 0) {
        fclose(w->fp);
        w->fp = NULL;
        return -1;
    }
    fputs(header, w->fp);

    w->t0 = rtde_ts;
    w->rows = 0;
    decimate_init(&w->next_emit, rtde_ts);
    printf("[RTDE] run started, logging to %s\n", w->path);
    return 0;
}

static void csv_write_sample(csv_writer_t *w, double rtde_ts,
                             const double force3[3], const double pose3[3])
{
    char row[256];

    if (!w->fp) {
        return;
    }
    if (!decimate_should_emit(rtde_ts, &w->next_emit, OUTPUT_GRID_S)) {
        return;
    }
    if (format_csv_row(row, sizeof(row), rtde_ts - w->t0, force3, pose3) < 0) {
        return;
    }
    fputs(row, w->fp);
    w->rows++;
}

/*
 * Flush and close.  Called on a clean end of run and equally on a disconnect,
 * so a trial interrupted by a controller reboot still leaves a complete,
 * parsable file on disk instead of a truncated one.
 */
static void csv_close(csv_writer_t *w)
{
    if (!w->fp) {
        return;
    }
    fflush(w->fp);
    fclose(w->fp);
    w->fp = NULL;
    printf("[RTDE] run finished, %ld rows in %s\n", w->rows, w->path);
}

/* ------------------------------------------------------------------ */
/* RTDE transport                                                      */
/* ------------------------------------------------------------------ */

#define MON_OK             0
#define MON_ERR_CONNECT   (-1)
#define MON_ERR_HANDSHAKE (-2)
#define MON_ERR_STREAM    (-3)

#define RECV_IDLE   1   /* nothing arrived within the socket timeout */
#define RECV_CLOSED 2   /* peer shut the connection down in an orderly way */

typedef struct {
    SOCKET sock;
    int protocol_version;
    uint8_t recipe_id;
} rtde_conn_t;

static int send_all(SOCKET s, const unsigned char *buf, int len)
{
    int sent = 0;

    while (sent < len) {
        int n = send(s, (const char *)buf + sent, len - sent, 0);
        if (n <= 0) {
            return -1;
        }
        sent += n;
    }
    return 0;
}

static int send_packet(SOCKET s, unsigned char type,
                       const unsigned char *payload, size_t n)
{
    unsigned char pkt[RTDE_MAX_PACKET];
    size_t total = RTDE_HEADER_SIZE + n;

    if (total > sizeof(pkt)) {
        return -1;
    }
    write_be_u16(pkt, (uint16_t)total);
    pkt[2] = type;
    if (n > 0) {
        memcpy(pkt + RTDE_HEADER_SIZE, payload, n);
    }
    return send_all(s, pkt, (int)total);
}

/*
 * Read exactly one packet.  A timeout while the socket is idle is reported as
 * RECV_IDLE rather than an error, which is what keeps Ctrl+C responsive and
 * distinguishes "the robot is quiet" from "the link is gone"; a timeout in
 * the middle of a packet would leave the stream out of frame, so it is an
 * error.
 */
static int recv_packet(SOCKET s, unsigned char *type,
                       unsigned char *payload, size_t *n)
{
    unsigned char hdr[RTDE_HEADER_SIZE];
    int got = 0;
    uint16_t size;

    while (got < RTDE_HEADER_SIZE) {
        int r = recv(s, (char *)hdr + got, RTDE_HEADER_SIZE - got, 0);
        if (r == 0) {
            /* An orderly shutdown between packets is the end of the stream,
             * not a fault; a reset link reports an error instead. */
            return (got == 0) ? RECV_CLOSED : -1;
        }
        if (r < 0) {
            if (got == 0 && WSAGetLastError() == WSAETIMEDOUT) {
                return RECV_IDLE;
            }
            return -1;
        }
        got += r;
    }
    size = read_be_u16(hdr);
    if (size < RTDE_HEADER_SIZE || size > RTDE_MAX_PACKET) {
        fprintf(stderr, "[RTDE] out-of-range packet size %u, stream out of frame\n",
                (unsigned)size);
        return -1;
    }
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

/*
 * Wait for one specific reply.  The controller emits RTDE_TEXT_MESSAGE
 * packets at its own initiative, including between a request and its answer,
 * so they are reported and skipped rather than mistaken for the reply - which
 * would abort a perfectly good handshake. An idle socket is retried, since
 * the 1 s receive timeout exists for Ctrl+C responsiveness, not as a deadline.
 */
static int recv_expect(SOCKET s, unsigned char want,
                       unsigned char *payload, size_t *n)
{
    int idle_rounds = 0;

    for (;;) {
        unsigned char type;
        int rc = recv_packet(s, &type, payload, n);

        if (rc == RECV_IDLE) {
            if (++idle_rounds > 10) {
                fprintf(stderr, "[RTDE] no answer to package type %u after "
                                "10 s\n", (unsigned)want);
                return -1;
            }
            continue;
        }
        if (rc != 0) {
            return -1;
        }
        if (type == want) {
            return 0;
        }
        if (type == RTDE_TEXT_MESSAGE) {
            printf("[RTDE] controller message during setup (%u bytes)\n",
                   (unsigned)*n);
            continue;
        }
        fprintf(stderr, "[RTDE] unexpected package type %u while waiting for "
                        "%u\n", (unsigned)type, (unsigned)want);
        return -1;
    }
}

/* Returns 1 accepted, 0 refused, -1 transport error. */
static int rtde_request_version(SOCKET s, uint16_t version)
{
    unsigned char req[2];
    unsigned char payload[RTDE_MAX_PACKET];
    size_t n;

    write_be_u16(req, version);
    if (send_packet(s, RTDE_REQUEST_PROTOCOL_VERSION, req, sizeof(req)) != 0) {
        return -1;
    }
    if (recv_expect(s, RTDE_REQUEST_PROTOCOL_VERSION, payload, &n) != 0 || n < 1) {
        return -1;
    }
    return payload[0] ? 1 : 0;
}

/* Returns 0 on success, -1 transport error, -2 the recipe was refused. */
static int rtde_setup_outputs(rtde_conn_t *c)
{
    unsigned char req[RTDE_MAX_PACKET];
    unsigned char payload[RTDE_MAX_PACKET];
    char types[512];
    size_t n, off = 0, len = strlen(RTDE_OUTPUT_RECIPE);

    if (c->protocol_version >= 2) {
        write_be_double(req, RTDE_REQUESTED_HZ);
        off = 8;
    }
    memcpy(req + off, RTDE_OUTPUT_RECIPE, len);
    if (send_packet(c->sock, RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, req, off + len) != 0) {
        return -1;
    }
    if (recv_expect(c->sock, RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, payload, &n) != 0) {
        return -1;
    }

    off = 0;
    if (c->protocol_version >= 2) {
        if (n < 1) {
            return -1;
        }
        c->recipe_id = payload[0];
        off = 1;
    }
    if (n - off >= sizeof(types)) {
        return -1;
    }
    memcpy(types, payload + off, n - off);
    types[n - off] = '\0';

    /*
     * A field this controller build does not export comes back as NOT_FOUND
     * in place of its type.  Refuse loudly here: continuing would decode the
     * payload at the wrong offsets and fill the CSV with plausible garbage.
     */
    if (strstr(types, "NOT_FOUND") != NULL) {
        fprintf(stderr,
                "[RTDE] controller refused part of the output recipe.\n"
                "       requested: %s\n"
                "       returned : %s\n"
                "       A NOT_FOUND entry means that field does not exist on "
                "this PolyScope build.\n", RTDE_OUTPUT_RECIPE, types);
        return -2;
    }
    if (strcmp(types, "DOUBLE,VECTOR6D,VECTOR6D,UINT32") != 0) {
        fprintf(stderr,
                "[RTDE] unexpected field types '%s' for recipe '%s'; refusing "
                "to decode at assumed offsets.\n", types, RTDE_OUTPUT_RECIPE);
        return -2;
    }
    return 0;
}

static int rtde_start(rtde_conn_t *c)
{
    unsigned char payload[RTDE_MAX_PACKET];
    size_t n;

    if (send_packet(c->sock, RTDE_CONTROL_PACKAGE_START, NULL, 0) != 0) {
        return -1;
    }
    if (recv_expect(c->sock, RTDE_CONTROL_PACKAGE_START, payload, &n) != 0 || n < 1) {
        return -1;
    }
    return payload[0] ? 0 : -1;
}

static int rtde_connect(rtde_conn_t *c, const char *ip, int port)
{
    struct sockaddr_in addr;
    DWORD timeout_ms = 1000;
    int accepted;

    memset(c, 0, sizeof(*c));
    c->sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (c->sock == INVALID_SOCKET) {
        fprintf(stderr, "[RTDE] socket() failed (%d)\n", WSAGetLastError());
        return MON_ERR_CONNECT;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((unsigned short)port);
    addr.sin_addr.s_addr = inet_addr(ip);
    if (addr.sin_addr.s_addr == INADDR_NONE) {
        fprintf(stderr, "[RTDE] '%s' is not a valid IPv4 address\n", ip);
        closesocket(c->sock);
        return MON_ERR_CONNECT;
    }
    if (connect(c->sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "[RTDE] cannot reach %s:%d (%d). Check the cable, the "
                        "static IP and that the robot is powered.\n",
                ip, port, WSAGetLastError());
        closesocket(c->sock);
        return MON_ERR_CONNECT;
    }
    setsockopt(c->sock, SOL_SOCKET, SO_RCVTIMEO,
               (const char *)&timeout_ms, sizeof(timeout_ms));

    /*
     * Version 2 first.  Version 1 is the fallback and is not a degraded mode
     * here: it carries no output_frequency field at all, which suits a tool
     * that takes whatever rate the controller streams and imposes its own
     * cadence from the packet timestamps.
     */
    accepted = rtde_request_version(c->sock, 2);
    if (accepted < 0) {
        fprintf(stderr, "[RTDE] protocol negotiation failed\n");
        closesocket(c->sock);
        return MON_ERR_HANDSHAKE;
    }
    if (accepted == 1) {
        c->protocol_version = 2;
    } else {
        accepted = rtde_request_version(c->sock, 1);
        if (accepted != 1) {
            fprintf(stderr, "[RTDE] controller accepted neither protocol "
                            "version 2 nor 1\n");
            closesocket(c->sock);
            return MON_ERR_HANDSHAKE;
        }
        c->protocol_version = 1;
    }

    if (rtde_setup_outputs(c) != 0) {
        closesocket(c->sock);
        return MON_ERR_HANDSHAKE;
    }
    if (rtde_start(c) != 0) {
        fprintf(stderr, "[RTDE] controller refused to start the output stream\n");
        closesocket(c->sock);
        return MON_ERR_HANDSHAKE;
    }

    printf("[RTDE] connected to %s:%d, protocol version %d\n",
           ip, port, c->protocol_version);
    return MON_OK;
}

/* ------------------------------------------------------------------ */
/* Monitor session                                                     */
/* ------------------------------------------------------------------ */

static volatile int g_stop = 0;

/*
 * One connection's worth of monitoring: connect, handshake, then read until
 * the peer goes away or the operator interrupts.  The CSV is closed on every
 * exit path, so no exit leaves a half-written file.
 */
static int monitor_run_once(const char *ip, int port, const char *out_dir)
{
    unsigned char payload[RTDE_MAX_PACKET];
    unsigned char type;
    rtde_conn_t conn;
    csv_writer_t writer;
    uint32_t prev_state = RT_UNKNOWN;
    double prev_ts = -1.0;
    int interval_reported = 0;
    size_t n;
    int rc;

    memset(&writer, 0, sizeof(writer));

    rc = rtde_connect(&conn, ip, port);
    if (rc != MON_OK) {
        return rc;
    }

    for (;;) {
        size_t off;
        double ts, pose3[3], force3[3];
        uint32_t state;
        file_action_t action;
        const unsigned char *body;
        int i;

        if (g_stop) {
            rc = MON_OK;
            break;
        }
        rc = recv_packet(conn.sock, &type, payload, &n);
        if (rc == RECV_IDLE) {
            continue;
        }
        if (rc == RECV_CLOSED) {
            printf("[RTDE] controller closed the stream.\n");
            rc = MON_OK;
            break;
        }
        if (rc != 0) {
            fprintf(stderr, "[RTDE] stream lost (%d). Data written so far is "
                            "kept.\n", WSAGetLastError());
            rc = MON_ERR_STREAM;
            break;
        }

        if (type == RTDE_TEXT_MESSAGE) {
            printf("[RTDE] controller message (%u bytes)\n", (unsigned)n);
            continue;
        }
        if (type != RTDE_DATA_PACKAGE) {
            continue;
        }

        off = (conn.protocol_version >= 2) ? 1 : 0;
        if (n < off + RTDE_PAYLOAD_SIZE) {
            fprintf(stderr, "[RTDE] short data package (%u bytes), skipped\n",
                    (unsigned)n);
            continue;
        }
        body = payload + off;

        ts = read_be_double(body + FIELD_OFF_TIMESTAMP);
        for (i = 0; i < 3; i++) {
            pose3[i]  = read_be_double(body + FIELD_OFF_TCP_POSE + 8 * i);
            force3[i] = read_be_double(body + FIELD_OFF_TCP_FORCE + 8 * i);
        }
        state = read_be_u32(body + FIELD_OFF_RUNTIME_STATE);

        /* Report the controller's real base rate once, since it is not
         * confirmed from public docs for this PolyScope build. */
        if (!interval_reported && prev_ts >= 0.0 && ts > prev_ts) {
            printf("[RTDE] stream interval %.4f s (%.1f Hz), decimating to "
                   TARGET_HZ_LABEL "\n", ts - prev_ts, 1.0 / (ts - prev_ts));
            interval_reported = 1;
        }
        prev_ts = ts;

        action = decide_file_action(prev_state, state);
        prev_state = state;
        if (action == FILE_ACTION_OPEN) {
            if (csv_open(&writer, out_dir, ip, port, ts) != 0) {
                rc = MON_ERR_STREAM;
                break;
            }
        } else if (action == FILE_ACTION_CLOSE) {
            csv_close(&writer);
            continue;   /* the STOPPED packet itself is not part of the run */
        }

        csv_write_sample(&writer, ts, force3, pose3);
    }

    csv_close(&writer);
    closesocket(conn.sock);
    return rc;
}

/* ------------------------------------------------------------------ */
/* Entry point                                                         */
/* ------------------------------------------------------------------ */

#ifndef RTDE_TEST_BUILD

static BOOL WINAPI console_handler(DWORD signal)
{
    (void)signal;
    g_stop = 1;
    printf("\n[RTDE] stopping, finalizing the current file...\n");
    return TRUE;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "RTDE fallback monitor for UR5 CB3\n"
        "\n"
        "  %s <robot-ip> <rtde-port> <out-dir>\n"
        "\n"
        "  example: %s 192.168.4.38 30004 .\n"
        "\n"
        "Reads the robot's RTDE output stream and writes one\n"
        "" CSV_FILE_PREFIX "YYYYMMDD_HHMMSS.csv per program run. Never sends a\n"
        "command to the robot. Stop it with Ctrl+C.\n", argv0, argv0);
}

int main(int argc, char **argv)
{
    WSADATA wsa;
    const char *ip, *out_dir;
    int port, rc;

    if (argc != 4) {
        usage(argv[0]);
        return 2;
    }
    ip = argv[1];
    port = atoi(argv[2]);
    out_dir = argv[3];
    if (!is_valid_ipv4(ip)) {
        fprintf(stderr, "[RTDE] '%s' is not a dotted-decimal IPv4 address "
                        "(expected e.g. 192.168.4.38)\n", ip);
        return 2;
    }
    if (port <= 0 || port > 65535) {
        fprintf(stderr, "[RTDE] '%s' is not a valid TCP port\n", argv[2]);
        return 2;
    }

    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "[RTDE] WSAStartup failed\n");
        return 1;
    }
    SetConsoleCtrlHandler(console_handler, TRUE);

    printf("[RTDE] fallback monitor watching %s:%d, writing to %s\n",
           ip, port, out_dir);
    printf("[RTDE] read-only: no command is ever sent to the robot\n");

    for (;;) {
        rc = monitor_run_once(ip, port, out_dir);
        if (g_stop) {
            break;
        }
        if (rc == MON_ERR_HANDSHAKE) {
            /* A refused recipe or protocol is a configuration fault; retrying
             * the same request forever would only hide it. */
            fprintf(stderr, "[RTDE] giving up: the controller refused the "
                            "session setup.\n");
            break;
        }
        printf("[RTDE] reconnecting in 2 s (Ctrl+C to stop)...\n");
        Sleep(2000);
    }

    WSACleanup();
    return (rc == MON_OK || g_stop) ? 0 : 1;
}

#endif /* RTDE_TEST_BUILD */
