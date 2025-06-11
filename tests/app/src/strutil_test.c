// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "strutil.h"

#include <zephyr/ztest.h>

ZTEST(strutil, test_split_at) {
  char str[] = "a,b";
  char* rem = split_at(str, ',');

  zassert_not_null(rem, "Split should return non-NULL");
  zassert_equal(str[0], 'a', "First part should be 'a'");
  zassert_equal(str[1], 0, "First part should be null-terminated");
  zassert_equal(rem[0], 'b', "Second part should be 'b'");
  zassert_equal(rem[1], 0, "Second part should be null-terminated");
}

ZTEST(strutil, test_split_at_dupe) {
  char str[] = "a,,b";
  char* rem = split_at(str, ',');

  zassert_not_null(rem, "Split should return non-NULL");
  zassert_equal(str[0], 'a', "First part should be 'a'");
  zassert_equal(str[1], 0, "First part should be null-terminated");
  zassert_equal(rem[0], ',', "Second part should be ','");
  zassert_equal(rem[1], 'b', "Second part should be 'b'");
  zassert_equal(rem[2], 0, "Second part should be null-terminated");
}

ZTEST(strutil, test_split_not_found) {
  char str[] = "a";
  char* rem = split_at(str, ',');

  zassert_is_null(rem, "Second part should be NULL.");
  zassert_equal(str[0], 'a', "First part should be 'a'");
  zassert_equal(str[1], 0, "First part should be null-terminated");
}

ZTEST(strutil, test_split_at_edge_cases) {
  // Test split_at("a.", '.') -> str="a", return=""
  char str1[] = "a.";
  char* rem1 = split_at(str1, '.');
  zassert_not_null(rem1, "Should return non-NULL for empty second part");
  zassert_equal(str1[0], 'a', "First part should be 'a'");
  zassert_equal(str1[1], 0, "First part should be null-terminated");
  zassert_equal(rem1[0], 0, "Second part should be empty string");

  // Test split_at("", '.') -> str="", return=NULL
  char str2[] = "";
  char* rem2 = split_at(str2, '.');
  zassert_is_null(rem2, "Should return NULL for empty input");
  zassert_equal(str2[0], 0, "First part should remain empty");
}

ZTEST(strutil, test_split_by_space_basic) {
  // Test split_by_space("a b") -> str="a", return="b"
  char str[] = "a b";
  char* rem = split_by_space(str);

  zassert_not_null(rem, "Should return non-NULL");
  zassert_equal(str[0], 'a', "First part should be 'a'");
  zassert_equal(str[1], 0, "First part should be null-terminated");
  zassert_equal(rem[0], 'b', "Second part should be 'b'");
  zassert_equal(rem[1], 0, "Second part should be null-terminated");
}

ZTEST(strutil, test_split_by_space_multiple_spaces) {
  // Test split_by_space("a  b") -> str="a", return="b"
  char str[] = "a  b";
  char* rem = split_by_space(str);

  zassert_not_null(rem, "Should return non-NULL");
  zassert_equal(str[0], 'a', "First part should be 'a'");
  zassert_equal(str[1], 0, "First part should be null-terminated");
  zassert_equal(rem[0], 'b', "Second part should be 'b'");
}

ZTEST(strutil, test_split_by_space_edge_cases) {
  // Test split_by_space("a ") -> str="a", return=NULL
  char str1[] = "a ";
  char* rem1 = split_by_space(str1);
  zassert_is_null(rem1, "Should return NULL when only whitespace follows");
  zassert_equal(str1[0], 'a', "First part should be 'a'");
  zassert_equal(str1[1], 0, "First part should be null-terminated");

  // Test split_by_space("a") -> str="a", return=NULL
  char str2[] = "a";
  char* rem2 = split_by_space(str2);
  zassert_is_null(rem2, "Should return NULL when no whitespace");
  zassert_equal(str2[0], 'a', "First part should be 'a'");

  // Test split_by_space("") -> str="", return=NULL
  char str3[] = "";
  char* rem3 = split_by_space(str3);
  zassert_is_null(rem3, "Should return NULL for empty string");
  zassert_equal(str3[0], 0, "First part should remain empty");

  // Test split_by_space(" ") -> str="", return=NULL
  char str4[] = " ";
  char* rem4 = split_by_space(str4);
  zassert_is_null(rem4, "Should return NULL for whitespace-only string");
  zassert_equal(str4[0], 0, "First part should be empty after split");
}

ZTEST(strutil, test_parse_int_valid) {
  int value;
  zassert_true(parse_int("123", &value), "Should parse valid int");
  zassert_equal(value, 123, "Value should be 123");

  zassert_true(parse_int("-456", &value), "Should parse negative int");
  zassert_equal(value, -456, "Value should be -456");

  zassert_true(parse_int("0", &value), "Should parse zero");
  zassert_equal(value, 0, "Value should be 0");
}

ZTEST(strutil, test_parse_int_invalid) {
  int value;
  zassert_false(parse_int("abc", &value), "Should reject non-numeric");
  zassert_false(parse_int("123x", &value), "Should reject partial numeric");
  zassert_false(parse_int("", &value), "Should reject empty string");
  zassert_false(parse_int(NULL, &value), "Should reject NULL string");
  zassert_false(parse_int("123", NULL), "Should reject NULL output");
}

ZTEST(strutil, test_parse_float_valid) {
  float value;
  zassert_true(parse_float("123.5", &value), "Should parse valid float");
  zassert_within(value, 123.5f, 0.001f, "Value should be 123.5");

  zassert_true(parse_float("-45.67", &value), "Should parse negative float");
  zassert_within(value, -45.67f, 0.001f, "Value should be -45.67");

  zassert_true(parse_float("0.0", &value), "Should parse zero float");
  zassert_within(value, 0.0f, 0.001f, "Value should be 0.0");

  zassert_true(parse_float("42", &value), "Should parse int as float");
  zassert_within(value, 42.0f, 0.001f, "Value should be 42.0");
}

ZTEST(strutil, test_parse_float_invalid) {
  float value;
  zassert_false(parse_float("abc", &value), "Should reject non-numeric");
  zassert_false(parse_float("12.3x", &value), "Should reject partial numeric");
  zassert_false(parse_float("", &value), "Should reject empty string");
  zassert_false(parse_float(NULL, &value), "Should reject NULL string");
  zassert_false(parse_float("12.3", NULL), "Should reject NULL output");
}

ZTEST_SUITE(strutil, NULL, NULL, NULL, NULL, NULL);
