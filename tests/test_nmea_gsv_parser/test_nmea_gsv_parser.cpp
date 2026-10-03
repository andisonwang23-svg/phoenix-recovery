#include <unity.h>

#include "logic/nmea_gsv_parser.h"

using logic::parseGsvSatellitesInView;

void parses_multiple_constellation_talkers() {
    TEST_ASSERT_EQUAL_INT(11, parseGsvSatellitesInView("$GPGSV,3,1,11,01,40,083,41*00"));
    TEST_ASSERT_EQUAL_INT(8, parseGsvSatellitesInView("$BDGSV,2,1,08,201,30,100,35*00"));
    TEST_ASSERT_EQUAL_INT(14, parseGsvSatellitesInView("$GNGSV,4,1,14,01,40,083,41*00"));
}

void rejects_non_gsv_and_malformed_fields() {
    TEST_ASSERT_EQUAL_INT(-1, parseGsvSatellitesInView("$GPGGA,123519,4807.038,N"));
    TEST_ASSERT_EQUAL_INT(-1, parseGsvSatellitesInView("$GPGSV,3,1,,01,40,083,41*00"));
    TEST_ASSERT_EQUAL_INT(-1, parseGsvSatellitesInView("not-nmea"));
}

void zero_visible_is_a_valid_diagnostic() {
    TEST_ASSERT_EQUAL_INT(0, parseGsvSatellitesInView("$GPGSV,1,1,00*00"));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(parses_multiple_constellation_talkers);
    RUN_TEST(rejects_non_gsv_and_malformed_fields);
    RUN_TEST(zero_visible_is_a_valid_diagnostic);
    return UNITY_END();
}
