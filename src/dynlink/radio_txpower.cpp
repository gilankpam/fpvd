/* radio_txpower.cpp — port of dl_backend_radio.c (run_iw + apply/applySafe).
 *
 * Uses posix_spawnp to run `iw dev <iface> set txpower fixed <mBm>`
 * where mBm = dBm * 100.  apply() is diff-based; applySafe() runs
 * unconditionally (watchdog fallback).
 */
#include "dynlink/radio_txpower.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <spawn.h>
#include <sys/wait.h>

extern char **environ;

namespace fpvd::dynlink {

int RadioTxpower::runIw(int8_t dBm) {
    char mBm_str[16];
    std::snprintf(mBm_str, sizeof(mBm_str), "%d", static_cast<int>(dBm) * 100);

    char *const argv[] = {
        const_cast<char *>("iw"),
        const_cast<char *>("dev"),
        const_cast<char *>(iface_.c_str()),
        const_cast<char *>("set"),
        const_cast<char *>("txpower"),
        const_cast<char *>("fixed"),
        mBm_str,
        nullptr,
    };

    pid_t pid;
    int rc_spawn = posix_spawnp(&pid, "iw", nullptr, nullptr, argv, environ);
    if (rc_spawn != 0) {
        return -1;
    }

    int status;
    if (waitpid(pid, &status, 0) < 0) {
        return -1;
    }

    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        return -1;
    }

    return 0;
}

int RadioTxpower::apply(int8_t dBm) {
    if (current_ && *current_ == dBm) {
        return 0;  // unchanged — diff suppressed
    }
    int rc = runIw(dBm);
    if (rc == 0) {
        current_ = dBm;
    }
    return rc;
}

int RadioTxpower::applySafe(int8_t dBm) {
    int rc = runIw(dBm);
    if (rc == 0) {
        current_ = dBm;
    }
    return rc;
}

} // namespace fpvd::dynlink
