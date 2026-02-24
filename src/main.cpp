/*
 * haptic_server.cpp
 * WebSocket bridge: TouchX (OpenHaptics HD API) -> Browser (Three.js)
 *
 * Architecture:
 *   1 kHz servo loop : position -> sphere collision -> force feedback
 *   ~60 Hz main loop : JSON state -> browser via WebSocket
 *   HTTP server       : serves web/index.html on port 8080
 */

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
/* bcrypt.h removed – SHA-1 implemented directly */

#include <HD/hd.h>

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <mutex>
#include <fstream>
#include <sstream>

#pragma comment(lib, "ws2_32.lib")
/* bcrypt.lib removed */

/* ================================================================
   Shared haptic state (servo thread -> main thread)
   ================================================================ */
static struct {
    double pos[3];
    double force[3];
    double contactPt[3];
    double penetration;
    int    buttons;
    bool   touching;
} g_haptic = {};

static std::mutex  g_mutex;
static const char* g_model = "Unknown";

/* Squishy oblate sphere (flat disc shape) at origin */
static constexpr double DISC_RXZ   = 35.0;    /* mm – horizontal radius */
static constexpr double DISC_RY    = 25.0;    /* mm – vertical (half-height) */
static constexpr double MAX_FORCE  = 3.3;     /* N */
static constexpr double SQUISHY_K  = 0.04;    /* N/mm^1.5 – soft rubber */
static double g_stiffScale = 1.0;             /* adjustable via WebSocket */

/* Timestamped log */
static void logMsg(const char* fmt, ...) {
    SYSTEMTIME st;
    GetLocalTime(&st);
    printf("[%02d:%02d:%02d.%03d] ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
    fflush(stdout);
}

/* Graceful shutdown */
static volatile bool g_running = true;
BOOL WINAPI consoleHandler(DWORD sig) {
    if (sig == CTRL_C_EVENT || sig == CTRL_BREAK_EVENT) { g_running = false; return TRUE; }
    return FALSE;
}

/* ================================================================
   Base64 encode
   ================================================================ */
static std::string b64Encode(const uint8_t* d, size_t n) {
    static const char T[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string r;
    unsigned v = 0; int b = -6;          /* unsigned to avoid shift UB */
    for (size_t i = 0; i < n; i++) {
        v = (v << 8) | d[i]; b += 8;
        while (b >= 0) { r += T[(v >> b) & 0x3F]; b -= 6; }
    }
    if (b > -6) r += T[((v << 8) >> (b + 8)) & 0x3F];
    while (r.size() & 3) r += '=';
    return r;
}

/* ================================================================
   SHA-1 (FIPS 180-4) – no external dependency
   ================================================================ */
static std::vector<uint8_t> sha1(const std::string& in) {
    uint32_t h0=0x67452301, h1=0xEFCDAB89, h2=0x98BADCFE,
             h3=0x10325476, h4=0xC3D2E1F0;

    /* pad: append 0x80, zeros, then 64-bit big-endian bit length */
    uint64_t bits = (uint64_t)in.size() * 8;
    std::vector<uint8_t> msg(in.begin(), in.end());
    msg.push_back(0x80);
    while (msg.size() % 64 != 56) msg.push_back(0);
    for (int i = 7; i >= 0; i--) msg.push_back((uint8_t)(bits >> (i * 8)));

    /* process 64-byte blocks */
    for (size_t off = 0; off < msg.size(); off += 64) {
        uint32_t w[80];
        for (int i = 0; i < 16; i++)
            w[i] = (uint32_t)msg[off+i*4]<<24 | (uint32_t)msg[off+i*4+1]<<16 |
                    (uint32_t)msg[off+i*4+2]<<8 | msg[off+i*4+3];
        for (int i = 16; i < 80; i++) {
            uint32_t t = w[i-3]^w[i-8]^w[i-14]^w[i-16];
            w[i] = (t<<1)|(t>>31);
        }
        uint32_t a=h0, b=h1, c=h2, d=h3, e=h4;
        for (int i = 0; i < 80; i++) {
            uint32_t f, k;
            if      (i<20) { f=(b&c)|(~b&d);           k=0x5A827999; }
            else if (i<40) { f=b^c^d;                   k=0x6ED9EBA1; }
            else if (i<60) { f=(b&c)|(b&d)|(c&d);       k=0x8F1BBCDC; }
            else           { f=b^c^d;                    k=0xCA62C1D6; }
            uint32_t tmp = ((a<<5)|(a>>27)) + f + e + k + w[i];
            e=d; d=c; c=(b<<30)|(b>>2); b=a; a=tmp;
        }
        h0+=a; h1+=b; h2+=c; h3+=d; h4+=e;
    }

    std::vector<uint8_t> hash(20);
    for (int i=0;i<4;i++) { hash[i]=(uint8_t)(h0>>(24-i*8));
        hash[4+i]=(uint8_t)(h1>>(24-i*8)); hash[8+i]=(uint8_t)(h2>>(24-i*8));
        hash[12+i]=(uint8_t)(h3>>(24-i*8)); hash[16+i]=(uint8_t)(h4>>(24-i*8)); }
    return hash;
}

/* ================================================================
   Low-level socket helpers
   ================================================================ */
static bool recvAll(SOCKET s, char* buf, int len) {
    int got = 0;
    while (got < len) {
        int n = recv(s, buf + got, len - got, 0);
        if (n <= 0) return false;
        got += n;
    }
    return true;
}

/* ================================================================
   WebSocket helpers
   ================================================================ */

/* Send a text frame (server -> client, unmasked) */
static bool wsSend(SOCKET s, const std::string& msg) {
    char frame[512];
    int  hlen = 0;
    frame[hlen++] = (char)0x81;            /* FIN | text */
    size_t sz = msg.size();
    if (sz <= 125) {
        frame[hlen++] = (char)sz;
    } else {
        frame[hlen++] = (char)126;
        frame[hlen++] = (char)(sz >> 8);
        frame[hlen++] = (char)(sz & 0xFF);
    }
    std::memcpy(frame + hlen, msg.data(), sz);
    int total = hlen + (int)sz;
    return send(s, frame, total, 0) == total;
}

/* Send a binary frame (server -> client, unmasked) */
static bool wsSendBin(SOCKET s, const void* data, size_t sz) {
    char frame[128];
    int  hlen = 0;
    frame[hlen++] = (char)0x82;            /* FIN | binary */
    if (sz <= 125) {
        frame[hlen++] = (char)sz;
    } else {
        frame[hlen++] = (char)126;
        frame[hlen++] = (char)(sz >> 8);
        frame[hlen++] = (char)(sz & 0xFF);
    }
    std::memcpy(frame + hlen, data, sz);
    int total = hlen + (int)sz;
    return send(s, frame, total, 0) == total;
}

/* Read one WebSocket frame.  Returns false on close / error. */
static bool wsRecvFrame(SOCKET s) {
    uint8_t h[2];
    if (!recvAll(s, (char*)h, 2)) return false;

    int      opcode = h[0] & 0x0F;
    bool     masked = (h[1] & 0x80) != 0;
    uint64_t plen   = h[1] & 0x7F;

    if (plen == 126) {
        uint8_t e[2];
        if (!recvAll(s, (char*)e, 2)) return false;
        plen = ((uint64_t)e[0] << 8) | e[1];
    } else if (plen == 127) {
        uint8_t e[8];
        if (!recvAll(s, (char*)e, 8)) return false;
        plen = 0;
        for (int i = 0; i < 8; i++) plen = (plen << 8) | e[i];
    }

    uint8_t mask[4] = {};
    if (masked && !recvAll(s, (char*)mask, 4)) return false;

    /* Drain payload */
    std::string payload((size_t)plen, '\0');
    if (plen > 0 && !recvAll(s, &payload[0], (int)plen)) return false;

    /* Unmask */
    if (masked) {
        for (size_t i = 0; i < payload.size(); i++)
            payload[i] ^= mask[i % 4];
    }

    /* Parse text frames for settings (e.g. {"k":0.5}) */
    if (opcode == 0x1 && !payload.empty()) {
        auto kp = payload.find("\"k\":");
        if (kp != std::string::npos) {
            double val = 0;
            if (std::sscanf(payload.c_str() + kp + 4, "%lf", &val) == 1) {
                if (val > 0.0 && val <= 10.0) g_stiffScale = val;
            }
        }
    }

    if (opcode == 0x8) {                       /* close */
        uint8_t cf[] = {0x88, 0x00};
        send(s, (char*)cf, 2, 0);
        return false;
    }
    if (opcode == 0x9) {                       /* ping -> pong */
        uint8_t pong[] = {0x8A, 0x00};
        send(s, (char*)pong, 2, 0);
    }
    return true;
}

/* Perform WebSocket upgrade handshake.  `req` is the raw HTTP request. */
static bool wsHandshake(SOCKET s, const std::string& req) {
    auto p = req.find("Sec-WebSocket-Key: ");
    if (p == std::string::npos) return false;
    auto s0 = p + 19;
    auto s1 = req.find("\r\n", s0);
    std::string key = req.substr(s0, s1 - s0);

    auto h = sha1(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11");
    std::string acc = b64Encode(h.data(), h.size());

    std::string resp =
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " + acc + "\r\n\r\n";
    int sent = 0;
    while (sent < (int)resp.size()) {
        int n = send(s, resp.c_str() + sent, (int)resp.size() - sent, 0);
        if (n <= 0) return false;
        sent += n;
    }
    return true;
}

/* ================================================================
   HD API servo callback (~1 kHz)
   ================================================================ */
static HDCallbackCode HDCALLBACK servoLoop(void*) {
    hdBeginFrame(hdGetCurrentDevice());

    HDdouble pos[3];
    hdGetDoublev(HD_CURRENT_POSITION, pos);

    HDint btn = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &btn);

    /* Oblate ellipsoid collision – flat disc shape at origin */
    double f[3] = {0, 0, 0};
    double cpt[3] = {0, 0, 0};
    double pen = 0.0;
    bool   touch = false;

    /* Scaled coordinates: map ellipsoid to unit sphere */
    double sx = pos[0] / DISC_RXZ;
    double sy = pos[1] / DISC_RY;
    double sz = pos[2] / DISC_RXZ;
    double sLen = std::sqrt(sx*sx + sy*sy + sz*sz);

    if (sLen < 1.0 && sLen > 1e-6) {
        /* Penetration: distance from surface in scaled space, then unscale */
        pen = (1.0 - sLen) * (DISC_RXZ < DISC_RY ? DISC_RXZ : DISC_RY);

        /* Gradient of ellipsoid (outward normal, not normalized) */
        double gx = pos[0] / (DISC_RXZ * DISC_RXZ);
        double gy = pos[1] / (DISC_RY  * DISC_RY);
        double gz = pos[2] / (DISC_RXZ * DISC_RXZ);
        double gLen = std::sqrt(gx*gx + gy*gy + gz*gz);
        if (gLen < 1e-8) gLen = 1e-8;

        double fmag = SQUISHY_K * g_stiffScale * pen * std::sqrt(pen);
        if (fmag > MAX_FORCE) fmag = MAX_FORCE;

        /* Force along ellipsoid normal (outward) */
        f[0] = fmag * gx / gLen;
        f[1] = fmag * gy / gLen;
        f[2] = fmag * gz / gLen;

        /* Contact point: project pos onto ellipsoid surface */
        double inv = 1.0 / sLen;
        cpt[0] = pos[0] * inv;   /* = sx/sLen * DISC_RXZ */
        cpt[1] = pos[1] * inv;
        cpt[2] = pos[2] * inv;
        /* But scale back: cpt is on unit sphere in scaled space → unscale */
        cpt[0] = sx * inv * DISC_RXZ;
        cpt[1] = sy * inv * DISC_RY;
        cpt[2] = sz * inv * DISC_RXZ;
        touch = true;
    }

    hdSetDoublev(HD_CURRENT_FORCE, f);
    hdEndFrame(hdGetCurrentDevice());

    {
        std::lock_guard<std::mutex> lk(g_mutex);
        std::memcpy(g_haptic.pos,      pos, sizeof pos);
        std::memcpy(g_haptic.force,    f,   sizeof f);
        std::memcpy(g_haptic.contactPt, cpt, sizeof cpt);
        g_haptic.penetration = pen;
        g_haptic.buttons     = btn;
        g_haptic.touching    = touch;
    }
    return HD_CALLBACK_CONTINUE;
}

/* ================================================================
   File / HTTP helpers
   ================================================================ */
static std::string loadFile(const char* path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static void httpReply(SOCKET s, int code, const char* ct, const std::string& body) {
    char hdr[512];
    snprintf(hdr, sizeof hdr,
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %zu\r\n"
        "Connection: close\r\n\r\n",
        code, code == 200 ? "OK" : "Not Found", ct, body.size());
    send(s, hdr, (int)std::strlen(hdr), 0);
    if (!body.empty()) send(s, body.data(), (int)body.size(), 0);
}

/* ================================================================
   main
   ================================================================ */
int main() {
    SetConsoleCtrlHandler(consoleHandler, TRUE);

    /* --- Winsock ------------------------------------------------ */
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa)) {
        printf("WSAStartup failed\n");
        return 1;
    }

    /* --- OpenHaptics -------------------------------------------- */
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(hdGetError())) {
        printf("ERROR: could not initialise haptic device.\n");
        printf("Make sure TouchX is connected and drivers are installed.\n");
        WSACleanup();
        return 1;
    }
    g_model = hdGetString(HD_DEVICE_MODEL_TYPE);
    printf("Device initialised: %s\n", g_model);

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    HDSchedulerHandle hServo =
        hdScheduleAsynchronous(servoLoop, nullptr, HD_MAX_SCHEDULER_PRIORITY);

    /* --- Load HTML ---------------------------------------------- */
    std::string html = loadFile("web/index.html");
    if (html.empty()) html = loadFile("../web/index.html");
    if (html.empty()) html = loadFile("../../web/index.html");
    if (html.empty())
        printf("WARNING: web/index.html not found – HTTP will return 404.\n");

    /* --- Server socket ------------------------------------------ */
    SOCKET srv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof opt);

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(8080);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(srv, (sockaddr*)&addr, sizeof addr) == SOCKET_ERROR) {
        printf("ERROR: bind failed (port 8080 in use?).\n");
        hdStopScheduler(); hdDisableDevice(hHD); WSACleanup();
        return 1;
    }
    listen(srv, 5);

    /* non-blocking accept */
    u_long nb = 1;
    ioctlsocket(srv, FIONBIO, &nb);

    printf("Listening on http://localhost:8080\n");

    /* --- Main loop ---------------------------------------------- */
    SOCKET    wsClient = INVALID_SOCKET;
    ULONGLONG lastSend = 0;
    int       sendCount = 0;

    while (g_running) {
        /* Single select() for read + write + exception */
        fd_set rd, wr, ex;
        FD_ZERO(&rd); FD_ZERO(&wr); FD_ZERO(&ex);
        FD_SET(srv, &rd);

        ULONGLONG now = GetTickCount64();
        bool wantSend = (wsClient != INVALID_SOCKET && now - lastSend >= 16);

        if (wsClient != INVALID_SOCKET) {
            FD_SET(wsClient, &rd);
            FD_SET(wsClient, &ex);
            if (wantSend) FD_SET(wsClient, &wr);
        }

        timeval tv{0, 16000};
        select(0, &rd, &wr, &ex, &tv);

        /* ---- Socket exception ---------------------------------- */
        if (wsClient != INVALID_SOCKET && FD_ISSET(wsClient, &ex)) {
            int soErr = 0; int soLen = sizeof soErr;
            getsockopt(wsClient, SOL_SOCKET, SO_ERROR, (char*)&soErr, &soLen);
            logMsg("WS disconnect: socket exception (SO_ERROR=%d, sent=%d)", soErr, sendCount);
            closesocket(wsClient);
            wsClient = INVALID_SOCKET;
        }

        /* ---- Accept new TCP connection ------------------------- */
        if (FD_ISSET(srv, &rd)) {
            SOCKET c = accept(srv, nullptr, nullptr);
            if (c != INVALID_SOCKET) {
                int one = 1;
                setsockopt(c, IPPROTO_TCP, TCP_NODELAY, (char*)&one, sizeof one);

                /* blocking recv with short timeout for HTTP request */
                u_long blk = 0;
                ioctlsocket(c, FIONBIO, &blk);
                DWORD tmo = 500;
                setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, (char*)&tmo, sizeof tmo);

                char buf[4096];
                int  n = recv(c, buf, sizeof(buf) - 1, 0);
                if (n > 0) {
                    buf[n] = '\0';
                    std::string req(buf, n);
                    std::string lower(req);
                    for (auto& ch : lower) ch = (char)tolower((unsigned char)ch);

                    if (lower.find("upgrade: websocket") != std::string::npos) {
                        if (wsHandshake(c, req)) {
                            if (wsClient != INVALID_SOCKET) {
                                logMsg("WS disconnect: replaced (sent=%d)", sendCount);
                                closesocket(wsClient);
                            }
                            wsClient = c;
                            /* Non-blocking for ALL future I/O – never toggle back */
                            u_long nonBlock = 1;
                            ioctlsocket(wsClient, FIONBIO, &nonBlock);
                            lastSend = GetTickCount64();
                            sendCount = 0;
                            logMsg("WebSocket client connected.");
                        } else {
                            closesocket(c);
                        }
                    } else {
                        bool isRoot  = req.find("GET / HTTP")       != std::string::npos;
                        bool isIndex = req.find("GET /index.html")  != std::string::npos;
                        if ((isRoot || isIndex) && !html.empty())
                            httpReply(c, 200, "text/html; charset=utf-8", html);
                        else
                            httpReply(c, 404, "text/plain", "Not found");
                        closesocket(c);
                    }
                } else {
                    closesocket(c);
                }
            }
        }

        /* ---- Incoming WebSocket frames ------------------------- */
        if (wsClient != INVALID_SOCKET && FD_ISSET(wsClient, &rd)) {
            /* Socket is non-blocking – peek won't block */
            char peek;
            int peekN = recv(wsClient, &peek, 1, MSG_PEEK);
            if (peekN == 0) {
                logMsg("WS disconnect: peer closed (sent=%d)", sendCount);
                closesocket(wsClient);
                wsClient = INVALID_SOCKET;
            } else if (peekN > 0) {
                if (!wsRecvFrame(wsClient)) {
                    int err = WSAGetLastError();
                    if (err != WSAEWOULDBLOCK) {
                        logMsg("WS disconnect: frame error (WSA=%d, sent=%d)", err, sendCount);
                        closesocket(wsClient);
                        wsClient = INVALID_SOCKET;
                    }
                }
            }
            /* peekN < 0 with WSAEWOULDBLOCK: spurious select, ignore */
        }

        /* ---- Send haptic state at ~60 Hz ----------------------- */
        now = GetTickCount64();
        if (wsClient != INVALID_SOCKET && wantSend && FD_ISSET(wsClient, &wr)) {
            lastSend = now;

            double p[3], f[3], cpt[3];
            double pen;
            int    btn;
            bool   touch;
            {
                std::lock_guard<std::mutex> lk(g_mutex);
                std::memcpy(p,   g_haptic.pos,      sizeof p);
                std::memcpy(f,   g_haptic.force,    sizeof f);
                std::memcpy(cpt, g_haptic.contactPt, sizeof cpt);
                pen   = g_haptic.penetration;
                btn   = g_haptic.buttons;
                touch = g_haptic.touching;
            }

            double packet[7];
            packet[0] = p[0]; packet[1] = p[1]; packet[2] = p[2];
            packet[3] = touch ? pen : 0.0;
            packet[4] = cpt[0]; packet[5] = cpt[1]; packet[6] = cpt[2];

            /* Build & send binary WS frame inline */
            char frame[64];
            frame[0] = (char)0x82;
            frame[1] = (char)sizeof(packet);
            std::memcpy(frame + 2, packet, sizeof(packet));
            int total = 2 + (int)sizeof(packet);

            int n = send(wsClient, frame, total, 0);
            if (n == total) {
                sendCount++;
            } else if (n == SOCKET_ERROR) {
                int err = WSAGetLastError();
                if (err != WSAEWOULDBLOCK) {
                    logMsg("WS disconnect: send failed (WSA=%d, sent=%d)", err, sendCount);
                    closesocket(wsClient);
                    wsClient = INVALID_SOCKET;
                }
            } else {
                logMsg("WS disconnect: partial send %d/%d (sent=%d)", n, total, sendCount);
                closesocket(wsClient);
                wsClient = INVALID_SOCKET;
            }
        }
    }

    /* --- Cleanup ------------------------------------------------ */
    printf("Shutting down...\n");
    hdStopScheduler();
    hdUnschedule(hServo);
    hdDisableDevice(hHD);
    if (wsClient != INVALID_SOCKET) closesocket(wsClient);
    closesocket(srv);
    WSACleanup();
    return 0;
}
