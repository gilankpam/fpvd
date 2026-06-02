#pragma once
#include <chrono>
#include <functional>
#include <httplib.h>
#include <string>
#include <thread>

namespace fpvd {

class HttpServer {
public:
    using Handler = std::function<void(const httplib::Request&, httplib::Response&)>;

    void get(const std::string& path, Handler h);
    void patch(const std::string& path, Handler h);
    void post(const std::string& path, Handler h);

    void listenInBackground(const std::string& host, int port);
    bool waitUntilReady(std::chrono::milliseconds timeout);
    void stop();

private:
    httplib::Server svr_;
    std::thread thr_;
};

} // namespace fpvd
