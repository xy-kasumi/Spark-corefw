// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "coords.h"

#include <math.h>
#include <zephyr/ztest.h>

// Test posp_dist()
ZTEST(coords, test_posp_dist_basic) {
  pos_phys_t a = {0, 0, 0};
  pos_phys_t b = {3, 4, 0};  // 3-4-5 triangle
  zassert_within(posp_dist(&a, &b), 5.0f, 1e-4f, "Distance should be 5.0");
}

ZTEST(coords, test_posp_dist_zero) {
  pos_phys_t a = {1, 2, 3};
  zassert_within(posp_dist(&a, &a), 0.0f, 1e-4f,
                 "Same point distance should be 0");
}

ZTEST(coords, test_posp_dist_3d) {
  pos_phys_t a = {0, 0, 0};
  pos_phys_t b = {1, 1, 1};
  zassert_within(posp_dist(&a, &b), sqrtf(3.0f), 1e-4f, "3D diagonal distance");
}

// Test posp_interp()
ZTEST(coords, test_posp_interp_midpoint) {
  pos_phys_t a = {0, 0, 0};
  pos_phys_t b = {10, 20, 30};
  pos_phys_t result;
  posp_interp(&a, &b, 0.5f, &result);
  zassert_within(result.x, 5.0f, 1e-4f, "X midpoint");
  zassert_within(result.y, 10.0f, 1e-4f, "Y midpoint");
  zassert_within(result.z, 15.0f, 1e-4f, "Z midpoint");
}

ZTEST(coords, test_posp_interp_extrapolate) {
  pos_phys_t a = {0, 0, 0};
  pos_phys_t b = {10, 10, 10};
  pos_phys_t result;
  posp_interp(&a, &b, -0.5f, &result);  // Extrapolate backwards
  zassert_within(result.x, -5.0f, 1e-4f, "X extrapolation");
  zassert_within(result.y, -5.0f, 1e-4f, "Y extrapolation");
  zassert_within(result.z, -5.0f, 1e-4f, "Z extrapolation");
}

ZTEST(coords, test_posp_interp_endpoints) {
  pos_phys_t a = {1, 2, 3};
  pos_phys_t b = {4, 5, 6};
  pos_phys_t result;

  posp_interp(&a, &b, 0.0f, &result);
  zassert_within(result.x, 1.0f, 1e-4f, "t=0 should be point a");

  posp_interp(&a, &b, 1.0f, &result);
  zassert_within(result.x, 4.0f, 1e-4f, "t=1 should be point b");
}

ZTEST_SUITE(coords, NULL, NULL, NULL, NULL, NULL);
