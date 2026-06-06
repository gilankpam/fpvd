#include "doctest.h"
#include "config/schema.hpp"

using fpvd::Config;
using nlohmann::json;

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
