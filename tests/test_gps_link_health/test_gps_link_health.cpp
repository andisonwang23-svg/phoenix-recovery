#include <cstdio>
#include <cstdlib>

#include "logic/gps_link_health.h"

static int failures = 0;

#define CHECK(name, condition) do { \
    if (!(condition)) { std::printf("FAIL: %s\n", name); ++failures; } \
    else { std::printf("PASS: %s\n", name); } \
} while (0)

int main() {
    CHECK("no NMEA is disconnected",
          !logic::gpsNmeaActive(5000, 0, 3000));
    CHECK("recent NMEA is connected",
          logic::gpsNmeaActive(5000, 3000, 3000));
    CHECK("stale NMEA is disconnected",
          !logic::gpsNmeaActive(7001, 3000, 3000));

    CHECK("fresh fix is usable",
          logic::gpsFixUsable(5000, 3000, 3000, true, 900, 5000));
    CHECK("old fix is rejected even after previously valid",
          !logic::gpsFixUsable(9001, 3000, 3000, true, 6000, 5000));
    CHECK("NMEA without satellite fix stays connected but not usable",
          logic::gpsNmeaActive(5000, 4000, 3000) &&
          !logic::gpsFixUsable(5000, 4000, 3000, false, 0, 5000));

    CHECK("silent module waits through startup grace",
          !logic::gpsSilentRecoveryDue(11000, 0, 0, 0, 12000, 30000));
    CHECK("silent module recovery is rate limited",
          !logic::gpsSilentRecoveryDue(20000, 0, 0, 0, 12000, 30000));
    CHECK("silent module eventually requests recovery",
          logic::gpsSilentRecoveryDue(30000, 0, 0, 0, 12000, 30000));
    CHECK("active NMEA prevents power cycling",
          !logic::gpsSilentRecoveryDue(35000, 34000, 34000, 0, 12000, 30000));

    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
