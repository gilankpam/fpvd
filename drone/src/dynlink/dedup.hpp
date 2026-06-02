#pragma once
#include <cstdint>
namespace fpvd::dynlink {

class Dedup {
public:
    bool check(uint32_t seq);   // true => drop (stale/dup). port dl_dedup_check
    void reset();               // port dl_dedup_reset
private:
    uint32_t lastSeq_{0};
    bool ever_{false};
};

} // namespace fpvd::dynlink
