#include "gcode.h"
#include <zephyr/ztest.h>

ZTEST(gcode, test_basic_g0_command) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G0", &parsed);

  zassert_true(result, "G0 command should parse successfully");
  zassert_equal(parsed.code, 0, "Code should be 0 for G0");
  zassert_equal(parsed.sub_code, -1, "Sub-code should be -1 for G0");
  zassert_false(parsed.has_x, "Should not have X coordinate");
  zassert_false(parsed.has_y, "Should not have Y coordinate");
  zassert_false(parsed.has_z, "Should not have Z coordinate");
}

ZTEST(gcode, test_g38_2_command) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G38.2", &parsed);

  zassert_true(result, "G38.2 command should parse successfully");
  zassert_equal(parsed.code, 38, "Code should be 38 for G38.2");
  zassert_equal(parsed.sub_code, 2, "Sub-code should be 2 for G38.2");
  zassert_false(parsed.has_x, "Should not have X coordinate");
  zassert_false(parsed.has_y, "Should not have Y coordinate");
  zassert_false(parsed.has_z, "Should not have Z coordinate");
}

ZTEST(gcode, test_g1_with_coordinates) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G1 X10.5 Y-20.3 Z5", &parsed);

  zassert_true(result, "G1 with coordinates should parse successfully");
  zassert_equal(parsed.code, 1, "Code should be 1 for G1");
  zassert_equal(parsed.sub_code, -1, "Sub-code should be -1 for G1");
  zassert_true(parsed.has_x, "Should have X coordinate");
  zassert_true(parsed.has_y, "Should have Y coordinate");
  zassert_true(parsed.has_z, "Should have Z coordinate");
  zassert_equal(parsed.x, 10.5f, "X should be 10.5");
  zassert_equal(parsed.y, -20.3f, "Y should be -20.3");
  zassert_equal(parsed.z, 5.0f, "Z should be 5.0");
}

ZTEST_SUITE(gcode, NULL, NULL, NULL, NULL, NULL);
