#include "http/server.hpp"

namespace fpvd {

void HttpServer::get(const std::string& p, Handler h) { svr_.Get(p, h); }
void HttpServer::patch(const std::string& p, Handler h) { svr_.Patch(p, h); }
void HttpServer::post(const std::string& p, Handler h) { svr_.Post(p, h); }

void HttpServer::listenInBackground(const std::string& host, int port) {
    thr_ = std::thread([this, host, port] { svr_.listen(host, port); });
}

bool HttpServer::waitUntilReady(std::chrono::milliseconds timeout) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (svr_.is_running())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

void HttpServer::stop() {
    svr_.stop();
    if (thr_.joinable())
        thr_.join();
}

} // namespace fpvd
