#include "gcode.h"
#include <zephyr/ztest.h>

ZTEST(gcode, test_basic_g0_command) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G0", &parsed);

  zassert_true(result, "G0 command should parse successfully");
  zassert_equal(parsed.command, 0, "Command should be 0 for G0");
  zassert_false(parsed.has_x, "Should not have X coordinate");
  zassert_false(parsed.has_y, "Should not have Y coordinate");
  zassert_false(parsed.has_z, "Should not have Z coordinate");
  // zassert_true(parsed.has_z, "Should not have Z coordinate");
}

ZTEST_SUITE(gcode, NULL, NULL, NULL, NULL, NULL);
