#include "dedup.hpp"

namespace fpvd::dynlink {

void Dedup::reset() {
    ever_ = false;
}

bool Dedup::check(uint32_t seq) {
    if (!ever_) {
        lastSeq_ = seq;
        ever_    = true;
        return false;
    }
    int32_t delta = static_cast<int32_t>(seq - lastSeq_);
    if (delta <= 0) return true;
    lastSeq_ = seq;
    return false;
}

} // namespace fpvd::dynlink
