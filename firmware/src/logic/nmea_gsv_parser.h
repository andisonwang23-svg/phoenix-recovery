#pragma once

#include <cstdint>

namespace logic {

// Return the number of satellites reported as visible by a validly framed GSV
// sentence, or -1 when the sentence is not a GSV sentence. The caller only
// invokes this after TinyGPSPlus has accepted the sentence checksum.
inline int parseGsvSatellitesInView(const char* sentence) {
    if (!sentence || sentence[0] != '$') return -1;

    // Standard NMEA talker sentences use five characters between '$' and the
    // first comma (for example GPGSV, BDGSV, GLGSV, or GNGSV).
    if (sentence[3] != 'G' || sentence[4] != 'S' || sentence[5] != 'V' ||
        sentence[6] != ',') return -1;

    const char* field = sentence + 7;
    // Skip total-message and message-number fields. The next field is the
    // total number of satellites in view for this talker.
    for (int skipped = 0; skipped < 2; ++skipped) {
        while (*field && *field != ',' && *field != '*' &&
               *field != '\r' && *field != '\n') ++field;
        if (*field != ',') return -1;
        ++field;
    }

    if (*field < '0' || *field > '9') return -1;
    int value = 0;
    while (*field >= '0' && *field <= '9') {
        value = value * 10 + (*field - '0');
        if (value > 99) return -1;
        ++field;
    }
    return value;
}

} // namespace logic
