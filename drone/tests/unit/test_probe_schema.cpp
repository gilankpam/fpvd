#include "doctest.h"
#include "config/schema.hpp"
#include "config/validate.hpp"

using fpvd::Config;
using nlohmann::json;
using fpvd::validate;

TEST_CASE("probe schema: absent block uses defaults") {
    Config c = json::parse(R"({"link":{}})").get<Config>();
    CHECK(c.probe.enabled == false);
    CHECK(c.probe.mcsList.empty());
    CHECK(c.probe.pps == 25);
    CHECK(c.probe.packetBytes == 1400);
    CHECK(c.probe.basePort == 50);
    CHECK(c.probe.baseFeedPort == 6700);
}

TEST_CASE("probe schema: parses and round-trips") {
    json j = json::parse(R"({"probe":{"enabled":true,"mcsList":[3,5,7],
        "pps":20,"packetBytes":1400,"basePort":50,"baseFeedPort":6700}})");
    Config c = j.get<Config>();
    CHECK(c.probe.enabled);
    CHECK(c.probe.mcsList == std::vector<int>{3, 5, 7});
    CHECK(c.probe.pps == 20);
    json out = c;
    CHECK(out["probe"] == j["probe"]);
}

static bool has_path(const std::vector<fpvd::ValidationError>& e, const std::string& p) {
    for (auto& v : e) if (v.path == p) return true;
    return false;
}

TEST_CASE("probe validate: disabled probe is always valid") {
    Config c{};                    // probe.enabled defaults false
    c.probe.mcsList = {99};        // garbage ignored while disabled
    CHECK(validate(c).empty());
}

TEST_CASE("probe validate: rejects bad enabled config") {
    Config c{};
    c.probe.enabled = true;
    c.probe.mcsList = {8};         // out of 0..7
    c.probe.pps = 0;               // < 1
    auto errs = validate(c);
    CHECK(has_path(errs, "probe.mcsList"));
    CHECK(has_path(errs, "probe.pps"));
}

TEST_CASE("probe validate: rejects reserved radio_port collision") {
    Config c{};
    c.probe.enabled = true;
    c.probe.mcsList = {5};
    c.probe.basePort = 32;         // collides with the tun radio_port
    CHECK(has_path(validate(c), "probe.basePort"));
}

TEST_CASE("probe validate: accepts a sane enabled config") {
    Config c{};
    c.probe.enabled = true;
    c.probe.mcsList = {3, 5, 7};
    auto errs = validate(c);
    for (auto& e : errs) CHECK(e.path.rfind("probe", 0) != 0);
}

TEST_CASE("probe validate: rejects duplicate MCS values") {
    Config c{};
    c.probe.enabled = true;
    c.probe.mcsList = {5, 5};
    CHECK(has_path(validate(c), "probe.mcsList"));
}

TEST_CASE("probe validate: rejects baseFeedPort=0 (below 1024)") {
    Config c{};
    c.probe.enabled = true;
    c.probe.mcsList = {5};
    c.probe.baseFeedPort = 0;
    CHECK(has_path(validate(c), "probe.baseFeedPort"));
}

TEST_CASE("probe validate: rejects baseFeedPort that overflows 65535") {
    Config c{};
    c.probe.enabled = true;
    c.probe.mcsList = {1, 2, 3};
    c.probe.baseFeedPort = 65534;  // 65534 + 2 = 65536 > 65535
    CHECK(has_path(validate(c), "probe.baseFeedPort"));
}
