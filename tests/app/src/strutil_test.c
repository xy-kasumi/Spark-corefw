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

ZTEST_SUITE(strutil, NULL, NULL, NULL, NULL, NULL);
