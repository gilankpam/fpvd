#include "doctest.h"
#include "supervise/beamforming.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

static fs::path makeIface(const fs::path& procBase, const std::string& iface,
                          bool withBfNode) {
    fs::create_directories(procBase / iface);
    std::ofstream(procBase / iface / "mac_addr") << "mac_addr=00:c0:ca:11:22:33\n";
    if (withBfNode) std::ofstream(procBase / iface / "bf_monitor_conf") << "";
    return procBase / iface;
}

static std::string readFile(const fs::path& p) {
    std::ifstream f(p); std::stringstream ss; ss << f.rdbuf(); return ss.str();
}

TEST_CASE("beamforming: resolveLocalMac prefers proc mac_addr, falls back to sysfs") {
    auto tmp = fs::temp_directory_path() / "fpvd-bf-mac";
    fs::remove_all(tmp);
    auto proc = tmp / "proc"; auto sys = tmp / "sys";
    fs::create_directories(proc / "wlan0");
    fs::create_directories(sys / "wlan0");
    std::ofstream(proc / "wlan0" / "mac_addr") << "addr=00:c0:ca:aa:bb:cc";
    std::ofstream(sys / "wlan0" / "address") << "de:ad:be:ef:00:01\n";
    CHECK(fpvd::resolveLocalMac(proc.string(), sys.string(), "wlan0")
          == "00:c0:ca:aa:bb:cc");

    fs::remove(proc / "wlan0" / "mac_addr");  // fall back to sysfs
    CHECK(fpvd::resolveLocalMac(proc.string(), sys.string(), "wlan0")
          == "de:ad:be:ef:00:01");
    fs::remove_all(tmp);
}

TEST_CASE("beamforming: unsupported when bf_monitor_conf absent") {
    auto tmp = fs::temp_directory_path() / "fpvd-bf-unsup";
    fs::remove_all(tmp);
    makeIface(tmp / "proc", "wlan0", /*withBfNode=*/false);
    fpvd::BeamformingController bf((tmp / "proc").string(),
                                   (tmp / "sys").string());
    fpvd::BfParams p; p.iface = "wlan0"; p.driver = "88XXau";
    p.remoteMac = "00:c0:ca:dd:ee:ff";
    bf.reconcile(true, p);
    auto s = bf.status();
    CHECK(s.requested == true);
    CHECK(s.state == fpvd::BfState::Unsupported);
    CHECK(s.localMac == "00:c0:ca:11:22:33");
    fs::remove_all(tmp);
}

TEST_CASE("beamforming: active writes init sequence; stop resets driver") {
    auto tmp = fs::temp_directory_path() / "fpvd-bf-active";
    fs::remove_all(tmp);
    auto ifd = makeIface(tmp / "proc", "wlan0", /*withBfNode=*/true);
    fpvd::BeamformingController bf((tmp / "proc").string(),
                                   (tmp / "sys").string());
    fpvd::BfParams p; p.iface = "wlan0"; p.driver = "8812eu";
    p.remoteMac = "00:c0:ca:dd:ee:ff"; p.width = 10;
    p.ackTimeout = 255; p.intervalMs = 5;
    bf.reconcile(true, p);

    auto s = bf.status();
    CHECK(s.state == fpvd::BfState::Active);
    CHECK(s.bw == 20);                                   // width 10 => modulation 20
    CHECK(readFile(ifd / "bf_monitor_conf") == "1 00:c0:ca:dd:ee:ff 0 0");
    CHECK(readFile(ifd / "ack_timeout") == "255");

    // Loop produces soundings.
    for (int i = 0; i < 100 && bf.status().soundingCount == 0; ++i)
        std::this_thread::sleep_for(5ms);
    CHECK(bf.status().soundingCount > 0);

    bf.stop();
    CHECK(readFile(ifd / "bf_monitor_conf") == "0 00:00:00:00:00:00 0 0");
    CHECK(readFile(ifd / "ack_timeout") == "33");
    CHECK(bf.status().state == fpvd::BfState::Disabled);
    fs::remove_all(tmp);
}

TEST_CASE("beamforming: reconcile is idempotent and disables on enabled=false") {
    auto tmp = fs::temp_directory_path() / "fpvd-bf-idem";
    fs::remove_all(tmp);
    makeIface(tmp / "proc", "wlan0", /*withBfNode=*/true);
    fpvd::BeamformingController bf((tmp / "proc").string(),
                                   (tmp / "sys").string());
    fpvd::BfParams p; p.iface = "wlan0"; p.driver = "8812eu";
    p.remoteMac = "00:c0:ca:dd:ee:ff"; p.intervalMs = 5;
    bf.reconcile(true, p);
    CHECK(bf.status().state == fpvd::BfState::Active);

    bf.reconcile(true, p);                 // identical => still active
    CHECK(bf.status().state == fpvd::BfState::Active);

    bf.reconcile(false, p);                // disable
    CHECK(bf.status().state == fpvd::BfState::Disabled);
    fs::remove_all(tmp);
}

TEST_CASE("beamforming: parseCbrRssi extracts the 4th rfinfo field") {
    // token:ndp0:ndp1:cbrrssi0:cbrrssi1:cbrsnr0:cbrsnr1
    CHECK(fpvd::parseCbrRssi("0:29:13:-48:-67:21:23") == -48);
    CHECK(fpvd::parseCbrRssi("0:22:22:0:0:0:0") == 0);
    CHECK(fpvd::parseCbrRssi("") == 0);
    CHECK(fpvd::parseCbrRssi("garbage") == 0);
}

TEST_CASE("beamforming: force re-writes conf even when params unchanged") {
    auto tmp = fs::temp_directory_path() / "fpvd-bf-force";
    fs::remove_all(tmp);
    auto ifd = makeIface(tmp / "proc", "wlan0", /*withBfNode=*/true);
    fpvd::BeamformingController bf((tmp / "proc").string(), (tmp / "sys").string());
    fpvd::BfParams p; p.iface = "wlan0"; p.driver = "8812eu";
    p.remoteMac = "00:c0:ca:dd:ee:ff"; p.intervalMs = 5;
    bf.reconcile(true, p);
    CHECK(bf.status().state == fpvd::BfState::Active);

    // Simulate a radio reset wiping the conf node, then a same-params reconcile.
    std::ofstream(ifd / "bf_monitor_conf", std::ios::trunc) << "WIPED";
    bf.reconcile(true, p, /*force=*/false);          // idempotent => NOT rewritten
    CHECK(readFile(ifd / "bf_monitor_conf") == "WIPED");
    bf.reconcile(true, p, /*force=*/true);           // force => rewritten
    CHECK(readFile(ifd / "bf_monitor_conf") == "1 00:c0:ca:dd:ee:ff 0 0");
    bf.stop();
    fs::remove_all(tmp);
}

TEST_CASE("beamforming: parseCbrToken extracts the 1st rfinfo field") {
    CHECK(fpvd::parseCbrToken("4:30:15:-48:-67:21:23") == 4);
    CHECK(fpvd::parseCbrToken("63:0:0:0:0:0:0") == 63);
    CHECK(fpvd::parseCbrToken("") == -1);
    CHECK(fpvd::parseCbrToken("garbage") == -1);
}

TEST_CASE("beamforming: cbrFresh drops false when the rfinfo token stops advancing") {
    auto tmp = fs::temp_directory_path() / "fpvd-bf-fresh";
    fs::remove_all(tmp);
    auto ifd = makeIface(tmp / "proc", "wlan0", /*withBfNode=*/true);
    // Frozen rfinfo: token never changes -> stale.
    std::ofstream(ifd / "bf_monitor_rfinfo", std::ios::trunc) << "4:30:15:-48:-67:21:23";

    fpvd::BeamformingController bf((tmp / "proc").string(), (tmp / "sys").string());
    fpvd::BfParams p; p.iface = "wlan0"; p.driver = "8812eu";
    p.remoteMac = "00:c0:ca:dd:ee:ff"; p.intervalMs = 5;
    bf.reconcile(true, p);

    std::this_thread::sleep_for(120ms);          // many ticks of a frozen token
    CHECK(bf.status().cbrFresh == false);         // detected stale

    // Advance the token -> fresh again (check within a few ticks of the change,
    // so this does not depend on the exact kCbrStaleTicks value).
    { std::ofstream f(ifd / "bf_monitor_rfinfo", std::ios::trunc);
      f << "9:30:15:-48:-67:21:23"; }
    std::this_thread::sleep_for(15ms);
    CHECK(bf.status().cbrFresh == true);          // recovered
    bf.stop();
    fs::remove_all(tmp);
}

TEST_CASE("beamforming: loop survives a transient trig write failure and self-heals") {
    auto tmp = fs::temp_directory_path() / "fpvd-bf-resilient";
    fs::remove_all(tmp);
    auto ifd = makeIface(tmp / "proc", "wlan0", /*withBfNode=*/true);
    // Make bf_monitor_trig a DIRECTORY so the loop's trig writes fail.
    fs::create_directory(ifd / "bf_monitor_trig");

    fpvd::BeamformingController bf((tmp / "proc").string(), (tmp / "sys").string());
    fpvd::BfParams p; p.iface = "wlan0"; p.driver = "8812eu";
    p.remoteMac = "00:c0:ca:dd:ee:ff"; p.intervalMs = 5;
    bf.reconcile(true, p);                       // arms (conf write ok); loop starts

    std::this_thread::sleep_for(60ms);           // loop has tried trig writes (all fail)
    CHECK(bf.status().state == fpvd::BfState::Error);   // in error...
    CHECK(bf.status().soundingCount == 0);              // ...no successful soundings

    // The loop must still be ALIVE: clear the failure and confirm it recovers.
    fs::remove(ifd / "bf_monitor_trig");         // remove the dir; writes now create a file
    std::this_thread::sleep_for(60ms);
    auto s = bf.status();
    CHECK(s.state == fpvd::BfState::Active);      // recovered (loop did NOT die)
    CHECK(s.soundingCount > 0);                   // sounding again
    bf.stop();
    fs::remove_all(tmp);
}
