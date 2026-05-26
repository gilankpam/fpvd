#include "doctest.h"
#include "supervise/radio.hpp"

TEST_CASE("radio: bring up captures driver/iface from stdout") {
    fpvd::Config c{};
    auto r = fpvd::bringUpRadio("tests/fixtures/fake_radio_up_ok.sh", c);
    REQUIRE(r.ok);
    CHECK(r.driver == "8812eu");
    CHECK(r.iface == "wlan0");
    CHECK(r.adapterId.value_or("") == "bl-m8812eu2");
}

TEST_CASE("radio: failure surfaces exit code and stderr") {
    fpvd::Config c{};
    auto r = fpvd::bringUpRadio("tests/fixtures/fake_radio_up_fail.sh", c);
    CHECK_FALSE(r.ok);
    CHECK(r.exitCode == 3);
    CHECK(r.stderrText.find("missing modules") != std::string::npos);
}
