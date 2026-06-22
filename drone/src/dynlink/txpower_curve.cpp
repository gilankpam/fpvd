/* txpower_curve.cpp — per-MCS TX power lookup (bl-m8812eu2 level 4). */
#include "dynlink/txpower_curve.hpp"

namespace fpvd::dynlink {

int8_t txpowerDbmForMcs(int mcs) {
    if (mcs < 0)
        mcs = 0;
    if (mcs > 7)
        mcs = 7;
    return kTxPowerDbmByMcs[mcs];
}

} // namespace fpvd::dynlink
