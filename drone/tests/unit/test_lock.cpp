#include "doctest.h"
#include "config/lock.hpp"
#include <nlohmann/json.hpp>

using fpvd::Config;
using fpvd::checkDynamicLinkLock;

static Config dlOn() {
    Config c{}; c.dynamicLink.enabled = true; return c;
}

TEST_CASE("lock: DL off → any path passes") {
    Config off{};
    auto body = nlohmann::json::parse(R"({"link":{"mcs":5}})");
    auto r = checkDynamicLinkLock(body, off);
    CHECK(r.ok);
    CHECK(r.lockedPaths.empty());
}

TEST_CASE("lock: DL on + body writes link.mcs → rejected") {
    auto body = nlohmann::json::parse(R"({"link":{"mcs":5}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "link.mcs");
}

TEST_CASE("lock: DL on + body writes link.fec.k → rejected") {
    auto body = nlohmann::json::parse(R"({"link":{"fec":{"k":4}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "link.fec.k");
}

TEST_CASE("lock: DL on + body overwrites link.fec wholesale → rejected") {
    auto body = nlohmann::json::parse(R"({"link":{"fec":{"k":4,"n":10}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    // Two children inside the locked subtree; either ordering is acceptable.
    REQUIRE(r.lockedPaths.size() == 2);
}

TEST_CASE("lock: DL on + body writes video.roi.qp → rejected") {
    auto body = nlohmann::json::parse(R"({"video":{"roi":{"qp":-10}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "video.roi.qp");
}

TEST_CASE("lock: DL on + body writes link.channel → allowed (not locked)") {
    auto body = nlohmann::json::parse(R"({"link":{"channel":165}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
}

TEST_CASE("lock: DL on + body writes dynamicLink.safe.mcs → allowed") {
    auto body = nlohmann::json::parse(R"({"dynamicLink":{"safe":{"mcs":3}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
}

TEST_CASE("lock: pending evaluated post-merge — body disables DL and writes locked key → allowed") {
    // Effective state has DL on; the body disables it AND writes link.mcs.
    // The caller must pre-compute the merged pending: in this case DL is off
    // after the merge, so the lock is open.
    Config mergedAfterPatch{}; // DL off (default)
    auto body = nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":false},"link":{"mcs":5}})");
    auto r = checkDynamicLinkLock(body, mergedAfterPatch);
    CHECK(r.ok);
}

TEST_CASE("lock: body enables DL and writes locked key → rejected") {
    // Merged pending has enabled=true; body wrote link.mcs in the same op.
    Config merged = dlOn();
    auto body = nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":true},"link":{"mcs":5}})");
    auto r = checkDynamicLinkLock(body, merged);
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "link.mcs");
}

TEST_CASE("lock: multiple locked paths reported together") {
    auto body = nlohmann::json::parse(
        R"({"link":{"mcs":5,"width":40},"video":{"bitrate":1000}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    CHECK(r.lockedPaths.size() == 3);
}

TEST_CASE("lock: body writes link.fec but with no children → still rejected") {
    // Wholesale write of the locked subtree as an empty object —
    // implementation detail: the walker stops at the empty object and
    // records the prefix `link.fec` itself, which trips the lock.
    auto body = nlohmann::json::parse(R"({"link":{"fec":{}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "link.fec");
}

TEST_CASE("lock: non-object body is allowed through regardless of DL") {
    auto body = nlohmann::json::parse(R"([1,2,3])");
    CHECK(checkDynamicLinkLock(body, dlOn()).ok);
}

TEST_CASE("lock: empty body is always allowed (DL on)") {
    CHECK(checkDynamicLinkLock(nlohmann::json::object(), dlOn()).ok);
}

TEST_CASE("lock: null leaf inside locked subtree is still rejected") {
    // {"link":{"fec":null}} writes the path link.fec with a null value.
    // null is a non-object leaf, so the walker emits the prefix link.fec,
    // which matches the link.fec lock and rejects.
    auto body = nlohmann::json::parse(R"({"link":{"fec":null}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "link.fec");
}

TEST_CASE("lock: DL on + body writes link.stbc → allowed (preserved, not DL-owned)") {
    // stbc/ldpc are static link params the controller preserves; the GS never
    // decides them, so an operator may retune them while DL is enabled.
    auto body = nlohmann::json::parse(R"({"link":{"stbc":true}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
    CHECK(r.lockedPaths.empty());
}

TEST_CASE("lock: DL on + body writes link.ldpc → allowed (preserved, not DL-owned)") {
    auto body = nlohmann::json::parse(R"({"link":{"ldpc":true}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
    CHECK(r.lockedPaths.empty());
}

TEST_CASE("lock: DL off + body writes link.stbc → allowed") {
    Config off{};
    auto body = nlohmann::json::parse(R"({"link":{"stbc":true}})");
    auto r = checkDynamicLinkLock(body, off);
    CHECK(r.ok);
    CHECK(r.lockedPaths.empty());
}

TEST_CASE("lock: DL off + body writes link.ldpc → allowed") {
    Config off{};
    auto body = nlohmann::json::parse(R"({"link":{"ldpc":true}})");
    auto r = checkDynamicLinkLock(body, off);
    CHECK(r.ok);
    CHECK(r.lockedPaths.empty());
}

TEST_CASE("lock: DL on + body writes link.txpower → allowed (operator-owned, not DL-decided)") {
    // The controller stopped deciding txpower in Phase 3a — tx power is constant,
    // applied at radio bring-up / hot-tuned via radio-tune, never written by the
    // decision loop. So an operator may set it while DL is enabled (like stbc/ldpc).
    auto body = nlohmann::json::parse(R"({"link":{"txpower":20}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
    CHECK(r.lockedPaths.empty());
}
