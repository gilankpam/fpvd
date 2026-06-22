#include "config/store.hpp"
#include "doctest.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
namespace fs = std::filesystem;

using fpvd::Config;

TEST_CASE("loadEffective: no file yields code defaults") {
    Config c = fpvd::loadEffective("/no/such/config.json");
    CHECK(c.dynamicLink.healthTimeoutMs == 10000); // schema default
    CHECK(c.link.width == 20);
}

TEST_CASE("loadEffective: present key overrides, missing key defaults") {
    auto tmp = std::filesystem::temp_directory_path() / "fpvd-cfg-load.json";
    std::ofstream(tmp) << R"({"dynamicLink":{"healthTimeoutMs":7000}})";
    Config c = fpvd::loadEffective(tmp.string());
    CHECK(c.dynamicLink.healthTimeoutMs == 7000); // from file
    CHECK(c.dynamicLink.applyStaggerMs == 50);    // missing -> default
    std::filesystem::remove(tmp);
}

TEST_CASE("loadEffective: malformed config throws") {
    CHECK_THROWS_AS(fpvd::loadEffective("tests/fixtures/malformed.json"), fpvd::StoreError);
}

TEST_CASE("unknownConfigKeys flags strays but not services entries") {
    nlohmann::json cfg = {
        {"dynamicLink", {{"bogusKnob", 1}}},
        {"services", {{"myproc", {{"exec", "/bin/true"}}}}},
        {"strayTop", true},
    };
    auto unknown = fpvd::unknownConfigKeys(cfg);
    CHECK(std::find(unknown.begin(), unknown.end(), "dynamicLink.bogusKnob") != unknown.end());
    CHECK(std::find(unknown.begin(), unknown.end(), "strayTop") != unknown.end());
    CHECK(std::find(unknown.begin(), unknown.end(), "services.myproc") == unknown.end());
}

TEST_CASE("store: deepMergeJson merges nested objects") {
    using nlohmann::json;
    json base = {{"a", {{"x", 1}, {"y", 2}}}, {"b", 3}};
    json over = {{"a", {{"y", 20}}}};
    json m = fpvd::deepMergeJson(base, over);
    CHECK(m["a"]["x"] == 1);
    CHECK(m["a"]["y"] == 20);
    CHECK(m["b"] == 3);
}

TEST_CASE("store: deepMergeJson replaces arrays wholesale") {
    using nlohmann::json;
    json base = {{"arr", {1, 2, 3}}};
    json over = {{"arr", {9}}};
    json m = fpvd::deepMergeJson(base, over);
    CHECK(m["arr"] == json::array({9}));
}

TEST_CASE("store: atomicWriteJson writes file and survives") {
    auto tmp = fs::temp_directory_path() / "fpvd_atomic_test.json";
    fs::remove(tmp);
    nlohmann::json j = {{"k", "v"}};
    fpvd::atomicWriteJson(tmp.string(), j);
    std::ifstream in(tmp);
    nlohmann::json round;
    in >> round;
    CHECK(round == j);
    CHECK_FALSE(fs::exists(tmp.string() + ".tmp"));
    fs::remove(tmp);
}

TEST_CASE("store: code defaults carry dynamicLink section") {
    auto c = fpvd::loadEffective("/no/such/path");
    CHECK(c.dynamicLink.enabled == false);
    CHECK(c.dynamicLink.roiQp.thresholdKbps == 6000);
}

TEST_CASE("loadEffective: wrong-typed known key throws StoreError") {
    auto tmp = std::filesystem::temp_directory_path() / "fpvd-cfg-wrongtype.json";
    std::ofstream(tmp) << R"({"link":{"channel":"not-a-number"}})";
    CHECK_THROWS_AS(fpvd::loadEffective(tmp.string()), fpvd::StoreError);
    std::filesystem::remove(tmp);
}
