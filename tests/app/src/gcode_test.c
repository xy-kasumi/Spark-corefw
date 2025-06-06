#include "gcode.h"
#include <zephyr/ztest.h>

ZTEST(gcode, test_basic_g0_command) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G0", &parsed);

  zassert_true(result, "G0 command should parse successfully");
  zassert_equal(parsed.code, 0, "Code should be 0 for G0");
  zassert_equal(parsed.sub_code, -1, "Sub-code should be -1 for G0");
  zassert_equal(parsed.x_state, AXIS_NOT_SPECIFIED,
                "X should not be specified");
  zassert_equal(parsed.y_state, AXIS_NOT_SPECIFIED,
                "Y should not be specified");
  zassert_equal(parsed.z_state, AXIS_NOT_SPECIFIED,
                "Z should not be specified");
}

ZTEST(gcode, test_g38_2_command) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G38.2", &parsed);

  zassert_true(result, "G38.2 command should parse successfully");
  zassert_equal(parsed.code, 38, "Code should be 38 for G38.2");
  zassert_equal(parsed.sub_code, 2, "Sub-code should be 2 for G38.2");
  zassert_equal(parsed.x_state, AXIS_NOT_SPECIFIED,
                "X should not be specified");
  zassert_equal(parsed.y_state, AXIS_NOT_SPECIFIED,
                "Y should not be specified");
  zassert_equal(parsed.z_state, AXIS_NOT_SPECIFIED,
                "Z should not be specified");
}

ZTEST(gcode, test_g1_with_coordinates) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G1 X10.5 Y-20.3 Z5", &parsed);

  zassert_true(result, "G1 with coordinates should parse successfully");
  zassert_equal(parsed.code, 1, "Code should be 1 for G1");
  zassert_equal(parsed.sub_code, -1, "Sub-code should be -1 for G1");
  zassert_equal(parsed.x_state, AXIS_WITH_VALUE, "X should have value");
  zassert_equal(parsed.y_state, AXIS_WITH_VALUE, "Y should have value");
  zassert_equal(parsed.z_state, AXIS_WITH_VALUE, "Z should have value");
  zassert_equal(parsed.x, 10.5f, "X should be 10.5");
  zassert_equal(parsed.y, -20.3f, "Y should be -20.3");
  zassert_equal(parsed.z, 5.0f, "Z should be 5.0");
}

ZTEST(gcode, test_g28_axis_only) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G28 X", &parsed);

  zassert_true(result, "G28 X should parse successfully");
  zassert_equal(parsed.code, 28, "Code should be 28 for G28");
  zassert_equal(parsed.sub_code, -1, "Sub-code should be -1 for G28");
  zassert_equal(parsed.x_state, AXIS_ONLY, "X should be axis only");
  zassert_equal(parsed.y_state, AXIS_NOT_SPECIFIED,
                "Y should not be specified");
  zassert_equal(parsed.z_state, AXIS_NOT_SPECIFIED,
                "Z should not be specified");
}

ZTEST_SUITE(gcode, NULL, NULL, NULL, NULL, NULL);
