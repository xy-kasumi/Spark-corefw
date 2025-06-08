// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
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

// M-code parsing tests
ZTEST(gcode, test_basic_m3_command) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("M3", &parsed);

  zassert_true(result, "M3 command should parse successfully");
  zassert_equal(parsed.cmd_type, CMD_TYPE_M, "Command type should be M");
  zassert_equal(parsed.code, 3, "Code should be 3 for M3");
  zassert_equal(parsed.sub_code, -1, "Sub-code should be -1 for M3");
  zassert_equal(parsed.p_state, PARAM_NOT_SPECIFIED,
                "P should not be specified");
  zassert_equal(parsed.q_state, PARAM_NOT_SPECIFIED,
                "Q should not be specified");
  zassert_equal(parsed.r_state, PARAM_NOT_SPECIFIED,
                "R should not be specified");
}

ZTEST(gcode, test_m3_with_all_parameters) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("M3 P750 Q1.5 R30", &parsed);

  zassert_true(result, "M3 with parameters should parse successfully");
  zassert_equal(parsed.cmd_type, CMD_TYPE_M, "Command type should be M");
  zassert_equal(parsed.code, 3, "Code should be 3 for M3");
  zassert_equal(parsed.sub_code, -1, "Sub-code should be -1 for M3");
  zassert_equal(parsed.p_state, PARAM_SPECIFIED, "P should be specified");
  zassert_equal(parsed.q_state, PARAM_SPECIFIED, "Q should be specified");
  zassert_equal(parsed.r_state, PARAM_SPECIFIED, "R should be specified");
  zassert_equal(parsed.p, 750.0f, "P should be 750.0");
  zassert_equal(parsed.q, 1.5f, "Q should be 1.5");
  zassert_equal(parsed.r, 30.0f, "R should be 30.0");
}

ZTEST(gcode, test_m4_with_partial_parameters) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("M4 Q2.0 R25", &parsed);

  zassert_true(result, "M4 with partial parameters should parse successfully");
  zassert_equal(parsed.cmd_type, CMD_TYPE_M, "Command type should be M");
  zassert_equal(parsed.code, 4, "Code should be 4 for M4");
  zassert_equal(parsed.sub_code, -1, "Sub-code should be -1 for M4");
  zassert_equal(parsed.p_state, PARAM_NOT_SPECIFIED,
                "P should not be specified");
  zassert_equal(parsed.q_state, PARAM_SPECIFIED, "Q should be specified");
  zassert_equal(parsed.r_state, PARAM_SPECIFIED, "R should be specified");
  zassert_equal(parsed.q, 2.0f, "Q should be 2.0");
  zassert_equal(parsed.r, 25.0f, "R should be 25.0");
}

ZTEST(gcode, test_m5_command) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("M5", &parsed);

  zassert_true(result, "M5 command should parse successfully");
  zassert_equal(parsed.cmd_type, CMD_TYPE_M, "Command type should be M");
  zassert_equal(parsed.code, 5, "Code should be 5 for M5");
  zassert_equal(parsed.sub_code, -1, "Sub-code should be -1 for M5");
  zassert_equal(parsed.p_state, PARAM_NOT_SPECIFIED,
                "P should not be specified");
  zassert_equal(parsed.q_state, PARAM_NOT_SPECIFIED,
                "Q should not be specified");
  zassert_equal(parsed.r_state, PARAM_NOT_SPECIFIED,
                "R should not be specified");
}

ZTEST(gcode, test_m_code_mixed_parameters) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("M3 P1000 R50", &parsed);

  zassert_true(result, "M3 with mixed parameters should parse successfully");
  zassert_equal(parsed.cmd_type, CMD_TYPE_M, "Command type should be M");
  zassert_equal(parsed.code, 3, "Code should be 3 for M3");
  zassert_equal(parsed.p_state, PARAM_SPECIFIED, "P should be specified");
  zassert_equal(parsed.q_state, PARAM_NOT_SPECIFIED,
                "Q should not be specified");
  zassert_equal(parsed.r_state, PARAM_SPECIFIED, "R should be specified");
  zassert_equal(parsed.p, 1000.0f, "P should be 1000.0");
  zassert_equal(parsed.r, 50.0f, "R should be 50.0");
}

ZTEST(gcode, test_invalid_m_code_parameter) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("M3 P", &parsed);

  zassert_false(result, "M3 with bare P should fail to parse");
}

ZTEST(gcode, test_m_code_with_unknown_parameter) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("M3 P500 S100", &parsed);

  zassert_false(result, "M3 with unknown parameter S should fail to parse");
}

ZTEST(gcode, test_invalid_m_code_number) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("M999", &parsed);

  zassert_true(
      result,
      "M999 should parse successfully (validation happens in execution)");
  zassert_equal(parsed.cmd_type, CMD_TYPE_M, "Command type should be M");
  zassert_equal(parsed.code, 999, "Code should be 999");
}

// Parse failure test cases
ZTEST(gcode, test_empty_string) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("", &parsed);

  zassert_false(result, "Empty string should fail to parse");
}

ZTEST(gcode, test_whitespace_only) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("   ", &parsed);

  zassert_false(result, "Whitespace-only string should fail to parse");
}

ZTEST(gcode, test_extra_whitespace_success) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G0   X10.5    Y20", &parsed);

  zassert_true(result, "Extra whitespace should be handled gracefully");
  zassert_equal(parsed.cmd_type, CMD_TYPE_G, "Command type should be G");
  zassert_equal(parsed.code, 0, "Code should be 0");
  zassert_equal(parsed.x_state, AXIS_WITH_VALUE, "X should have value");
  zassert_equal(parsed.y_state, AXIS_WITH_VALUE, "Y should have value");
  zassert_equal(parsed.x, 10.5f, "X should be 10.5");
  zassert_equal(parsed.y, 20.0f, "Y should be 20.0");
}

ZTEST(gcode, test_lowercase_command) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("g0 X10", &parsed);

  zassert_false(result, "Lowercase command should fail to parse");
}

ZTEST(gcode, test_lowercase_parameter) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G0 x10", &parsed);

  zassert_false(result, "Lowercase parameter should fail to parse");
}

ZTEST(gcode, test_garbled_command) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G0abc X10", &parsed);

  zassert_false(result, "Garbled command should fail to parse");
}

ZTEST(gcode, test_garbled_number) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G0 X10.5.2", &parsed);

  zassert_false(result, "Garbled number should fail to parse");
}

ZTEST(gcode, test_no_whitespace_between_params) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("G0X1Y2", &parsed);

  zassert_false(result,
                "No whitespace between parameters should fail to parse");
}

ZTEST(gcode, test_invalid_decimal_format) {
  gcode_parsed_t parsed;
  bool result = parse_gcode("M3 P10..5", &parsed);

  zassert_false(result,
                "Invalid decimal format (double decimal) should fail to parse");
}

ZTEST_SUITE(gcode, NULL, NULL, NULL, NULL, NULL);
