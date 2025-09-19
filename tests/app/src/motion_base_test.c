// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "motion_base.h"

#include <zephyr/ztest.h>

// Test path_buffer_t initialization
ZTEST(motion_base, test_pb_init_basic) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {10, 0, 0};

  pb_init(&pb, &src, &dst);

  pos_phys_t pos = pb_get_pos(&pb);
  zassert_within(pos.x, 0.0f, 1e-4f, "initial pos");
  zassert_true(pb_can_write(&pb), "buffer available after construction");
  zassert_false(pb_at_end(&pb), "initial pos is not end");
}

// Test path_buffer_t movement
ZTEST(motion_base, test_pb_move_forward_simple) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {1, 0, 0};  // 1mm segment

  pb_init(&pb, &src, &dst);

  // Move 0.5mm forward
  zassert_true(pb_move(&pb, 0.5f), "move forward should succeed");
  pos_phys_t pos = pb_get_pos(&pb);
  zassert_within(pos.x, 0.5f, EDM_RESOLUTION_MM + 1e-4f, "should be at 0.5mm");
}

ZTEST(motion_base, test_pb_move_backward) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {1, 0, 0};

  pb_init(&pb, &src, &dst);

  // Move forward then backward
  pb_move(&pb, 0.5f);
  zassert_true(pb_move(&pb, -0.2f), "move backward should succeed");
  pos_phys_t pos = pb_get_pos(&pb);
  zassert_within(pos.x, 0.3f, EDM_RESOLUTION_MM + 1e-4f,
                 "should be at 0.3mm (+0.5mm - 0.2mm)");
}

ZTEST(motion_base, test_pb_move_retraction_limit) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {10, 0, 0};

  pb_init(&pb, &src, &dst);

  // Move forward much more than history size can track
  // EDM_HISTORY_SIZE=201, EDM_RESOLUTION_MM=0.005, so max history ~1mm
  pb_move(&pb, 5.0f);  // Move 5mm forward (way beyond history)

  // Try to retract way beyond limit - should fail
  zassert_false(pb_move(&pb, -10.0f),
                "Retraction beyond history limit should fail");
}

ZTEST(motion_base, test_pb_move_to_end) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {0.5f, 0, 0};  // Short segment

  pb_init(&pb, &src, &dst);

  // Move beyond current end
  pb_move(&pb, 1.0f);  // Try to move 1mm (> 0.5mm)

  zassert_true(pb_at_end(&pb), "Should be at end after overshooting");
  pos_phys_t pos = pb_get_pos(&pb);
  zassert_within(pos.x, 0.5f, EDM_RESOLUTION_MM + 1e-4f,
                 "Should be clipped to segment end");
}

// Test path_buffer_t multi-segment
ZTEST(motion_base, test_pb_write_and_traverse) {
  path_buffer_t pb;
  pos_phys_t p1 = {0, 0, 0};
  pos_phys_t p2 = {1, 0, 0};
  pos_phys_t p3 = {1, 1, 0};  // L-shaped path

  pb_init(&pb, &p1, &p2);

  zassert_true(pb_can_write(&pb), "Should be able to write to non-end segment");
  pb_write(&pb, &p3);

  // Move through both segments
  pb_move(&pb, 1.5f);  // Should be halfway through second segment
  pos_phys_t pos = pb_get_pos(&pb);
  zassert_within(pos.x, 1.0f, EDM_RESOLUTION_MM + 1e-4f,
                 "X must be middle of p2-p3 segment");
  zassert_within(pos.y, 0.5f, EDM_RESOLUTION_MM + 1e-4f,
                 "Y must be middle of p2-p3 segment");
}

ZTEST(motion_base, test_pb_write_buffer_full) {
  path_buffer_t pb;
  pos_phys_t p1 = {0, 0, 0};
  pos_phys_t p2 = {1, 0, 0};
  pos_phys_t p3 = {2, 0, 0};

  pb_init(&pb, &p1, &p2);
  pb_write(&pb, &p3);  // Fill the buffer
  zassert_false(pb_can_write(&pb), "buffer should be full");

  // Move to consume the buffered segment
  pb_move(&pb, 1.1f);  // Move past first segment
  zassert_true(pb_can_write(&pb), "first segment must be consumed");
}

// Test edge cases
ZTEST(motion_base, test_pb_tiny_movements) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {1, 0, 0};

  pb_init(&pb, &src, &dst);

  // Move less than EDM_RESOLUTION_MM - should not change position
  pos_phys_t pos_before = pb_get_pos(&pb);
  pb_move(&pb, EDM_RESOLUTION_MM * 0.5f);
  pos_phys_t pos_after = pb_get_pos(&pb);

  zassert_within(pos_before.x, pos_after.x, 1e-4f,
                 "Tiny movement should not change discrete position");
}

ZTEST(motion_base, test_pb_zero_length_segment) {
  path_buffer_t pb;
  pos_phys_t same = {5, 5, 5};

  pb_init(&pb, &same, &same);  // Zero-length segment

  pb_move(&pb, 1.0f);  // Should not crash
  zassert_true(pb_at_end(&pb), "Zero-length segment should be at end");

  pos_phys_t pos = pb_get_pos(&pb);
  zassert_within(pos.x, 5.0f, 1e-4f, "Should stay at same position");
}

ZTEST(motion_base, test_pb_accumulated_tiny_movements) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {1, 0, 0};

  pb_init(&pb, &src, &dst);

  // Accumulate tiny movements until they add up to one notch
  float tiny = EDM_RESOLUTION_MM * 0.3f;
  pb_move(&pb, tiny);  // 0.3 * 0.005 = 0.0015mm
  pb_move(&pb, tiny);  // 0.6 * 0.005 = 0.003mm
  pb_move(&pb, tiny);  // 0.9 * 0.005 = 0.0045mm
  pb_move(&pb, tiny);  // 1.2 * 0.005 = 0.006mm -> should trigger one notch

  pos_phys_t pos = pb_get_pos(&pb);
  zassert_true(pos.x >= EDM_RESOLUTION_MM - 1e-4f,
               "Accumulated tiny movements should eventually advance position");
}

ZTEST_SUITE(motion_base, NULL, NULL, NULL, NULL, NULL);
