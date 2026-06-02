#pragma once
#include "daemon.hpp"
#include "http/server.hpp"

namespace fpvd {

// Register all HTTP endpoints on `srv` that operate on `d`.
// `reallyRestart` is forwarded to Daemon::apply (false in tests).
void registerHandlers(HttpServer& srv, Daemon& d, bool reallyRestart);

} // namespace fpvd
