#include "daemon.hpp"
#include "http/handlers.hpp"
#include "http/server.hpp"
#include <atomic>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <thread>
#include <unistd.h>

static std::atomic<bool> g_stop{false};

static void onSignal(int) { g_stop.store(true); }

int main(int argc, char** argv) {
    std::string defaultsPath = "/rom/etc/fpvd/defaults.json";
    std::string overlayPath  = "/etc/fpvd/config.json";
    std::string radioUp      = "/usr/libexec/fpvd/radio-up.sh";
    std::string waybeamPath  = "/etc/waybeam.json";
    std::string httpHost     = "0.0.0.0";
    int httpPort             = 8080;
    std::string logPath;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--defaults" && i + 1 < argc) defaultsPath = argv[++i];
        else if (a == "--overlay" && i + 1 < argc) overlayPath = argv[++i];
        else if (a == "--radio-up" && i + 1 < argc) radioUp = argv[++i];
        else if (a == "--waybeam-json" && i + 1 < argc) waybeamPath = argv[++i];
        else if (a == "--host" && i + 1 < argc) httpHost = argv[++i];
        else if (a == "--port" && i + 1 < argc) httpPort = std::stoi(argv[++i]);
        else if (a == "--log" && i + 1 < argc) logPath = argv[++i];
        else if (a == "-h" || a == "--help") {
            std::cerr << "Usage: fpvd [--defaults PATH] [--overlay PATH] "
                         "[--radio-up PATH] [--waybeam-json PATH] "
                         "[--host HOST] [--port PORT] [--log PATH]\n";
            return 0;
        }
    }

    std::signal(SIGTERM, onSignal);
    std::signal(SIGINT,  onSignal);
    std::signal(SIGPIPE, SIG_IGN);

    if (!logPath.empty()) {
        int fd = ::open(logPath.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
        if (fd >= 0) {
            ::dup2(fd, 1);
            ::dup2(fd, 2);
            ::close(fd);
        } else {
            std::cerr << "fpvd: failed to open log " << logPath
                      << ": " << std::strerror(errno) << "\n";
        }
    }

    fpvd::DaemonPaths paths{defaultsPath, overlayPath, radioUp, waybeamPath};
    fpvd::Daemon daemon(paths);
    try {
        daemon.bootstrap(/*startProcesses=*/true);
    } catch (const std::exception& e) {
        std::cerr << "fpvd: bootstrap failed: " << e.what() << "\n";
        return 1;
    }

    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, daemon, /*reallyRestart=*/true);
    srv.listenInBackground(httpHost, httpPort);
    if (!srv.waitUntilReady(std::chrono::seconds(3))) {
        std::cerr << "fpvd: HTTP bind failed on " << httpHost << ":" << httpPort << "\n";
        return 2;
    }
    std::cerr << "fpvd: listening on " << httpHost << ":" << httpPort << "\n";

    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cerr << "fpvd: shutting down\n";
    srv.stop();
    daemon.orchestrator().stopAll();
    return 0;
}
