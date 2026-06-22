#include "doctest.h"
#include "http/server.hpp"
#include <httplib.h>

TEST_CASE("http: serves a registered GET endpoint") {
    fpvd::HttpServer srv;
    srv.get("/ping", [](const httplib::Request&, httplib::Response& res) {
        res.set_content("pong", "text/plain");
    });
    srv.listenInBackground("127.0.0.1", 18080);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client cli("http://127.0.0.1:18080");
    auto r = cli.Get("/ping");
    REQUIRE(r);
    CHECK(r->status == 200);
    CHECK(r->body == "pong");
    srv.stop();
}
