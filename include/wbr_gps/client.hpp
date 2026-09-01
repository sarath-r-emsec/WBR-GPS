#pragma once

#include "wbr_gps/gps_types.hpp"

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace wbr_gps {

// Talks to wbr-gpsd over a Unix socket. Deliberately has NO device access:
// if the daemon is unavailable, snapshot().service_ok is false and that is
// the whole story. There is no fallback path, by design (guarantee G5).
class Client {
public:
    ~Client();

    // Connect and keep a background thread maintaining a local cache. Returns
    // true once the thread is running; it returns true even if the daemon is
    // not up yet, because the thread will keep retrying. Check
    // snapshot().service_ok to learn whether data is actually flowing.
    bool start(const std::string& sock_path = kDefaultSocketPath,
               bool want_nmea = false);
    void stop();

    // Lock-guarded read of the cached snapshot. Cheap: no syscall. Safe to
    // call from a hot loop, which is how gsm_monitor uses it.
    Snapshot snapshot() const;

    // Raw NMEA sentences, when started with want_nmea = true. Called from the
    // background thread.
    void set_nmea_callback(std::function<void(const std::string&)> cb);

private:
    void run();
    void mark_unavailable();

    std::string             sock_path_;
    bool                    want_nmea_ = false;
    std::atomic<bool>       stopping_{ false };
    std::atomic<bool>       running_{ false };
    std::thread             thread_;
    mutable std::mutex      mu_;
    Snapshot                cached_;
    std::function<void(const std::string&)> nmea_cb_;
};

// One synchronous request. For startup checks such as an external-reference
// decision made before any long-lived client exists. Returns a default
// Snapshot with service_ok = false if the daemon is down.
Snapshot get_once(const std::string& sock_path = kDefaultSocketPath,
                  int timeout_ms = 500);

} // namespace wbr_gps
