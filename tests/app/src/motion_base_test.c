#include <zephyr/ztest.h>
#include <motion_base.h>
#include <math.h>

// Test posp_dist()
ZTEST(motion_base, test_posp_dist_basic) {
  pos_phys_t a = {0, 0, 0};
  pos_phys_t b = {3, 4, 0};  // 3-4-5 triangle
  zassert_within(posp_dist(&a, &b), 5.0f, 1e-4f, "Distance should be 5.0");
}

ZTEST(motion_base, test_posp_dist_zero) {
  pos_phys_t a = {1, 2, 3};
  zassert_within(posp_dist(&a, &a), 0.0f, 1e-4f, "Same point distance should be 0");
}

ZTEST(motion_base, test_posp_dist_3d) {
  pos_phys_t a = {0, 0, 0};
  pos_phys_t b = {1, 1, 1};
  zassert_within(posp_dist(&a, &b), sqrtf(3.0f), 1e-4f, "3D diagonal distance");
}

// Test posp_interp()
ZTEST(motion_base, test_posp_interp_midpoint) {
  pos_phys_t a = {0, 0, 0};
  pos_phys_t b = {10, 20, 30};
  pos_phys_t result;
  posp_interp(&a, &b, 0.5f, &result);
  zassert_within(result.x, 5.0f, 1e-4f, "X midpoint");
  zassert_within(result.y, 10.0f, 1e-4f, "Y midpoint");
  zassert_within(result.z, 15.0f, 1e-4f, "Z midpoint");
}

ZTEST(motion_base, test_posp_interp_extrapolate) {
  pos_phys_t a = {0, 0, 0};
  pos_phys_t b = {10, 10, 10};
  pos_phys_t result;
  posp_interp(&a, &b, -0.5f, &result);  // Extrapolate backwards
  zassert_within(result.x, -5.0f, 1e-4f, "X extrapolation");
  zassert_within(result.y, -5.0f, 1e-4f, "Y extrapolation");
  zassert_within(result.z, -5.0f, 1e-4f, "Z extrapolation");
}

ZTEST(motion_base, test_posp_interp_endpoints) {
  pos_phys_t a = {1, 2, 3};
  pos_phys_t b = {4, 5, 6};
  pos_phys_t result;
  
  posp_interp(&a, &b, 0.0f, &result);
  zassert_within(result.x, 1.0f, 1e-4f, "t=0 should be point a");
  
  posp_interp(&a, &b, 1.0f, &result);
  zassert_within(result.x, 4.0f, 1e-4f, "t=1 should be point b");
}

// Test path_buffer_t initialization
ZTEST(motion_base, test_pb_init_basic) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {10, 0, 0};
  
  pb_init(&pb, &src, &dst, false);
  
  pos_phys_t pos = pb_get_pos(&pb);
  zassert_within(pos.x, 0.0f, 1e-4f, "Initial position should be src");
  zassert_false(pb_is_ready(&pb), "Not ready - needs next segment or end marker");
  zassert_true(pb_can_write(&pb), "Should be able to write next segment");
  zassert_false(pb_at_end(&pb), "Should not be at end initially");
}

ZTEST(motion_base, test_pb_init_end_segment) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {1, 0, 0};
  
  pb_init(&pb, &src, &dst, true);  // End segment
  
  zassert_false(pb_can_write(&pb), "Cannot write to end segment");
  zassert_true(pb_is_ready(&pb), "End segment should be ready");
}

// Test path_buffer_t movement
ZTEST(motion_base, test_pb_move_forward_simple) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {1.0f, 0, 0};  // 1mm segment
  
  pb_init(&pb, &src, &dst, true);
  
  // Move 0.5mm forward
  zassert_true(pb_move(&pb, 0.5f), "Forward move should succeed");
  pos_phys_t pos = pb_get_pos(&pb);
  zassert_within(pos.x, 0.5f, EDM_RESOLUTION_MM + 1e-4f, "Should be at 0.5mm");
}

ZTEST(motion_base, test_pb_move_backward) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {1.0f, 0, 0};
  
  pb_init(&pb, &src, &dst, true);
  
  // Move forward then back
  pb_move(&pb, 0.5f);
  zassert_true(pb_move(&pb, -0.2f), "Backward move should succeed");
  pos_phys_t pos = pb_get_pos(&pb);
  zassert_within(pos.x, 0.3f, EDM_RESOLUTION_MM + 1e-4f, "Should be at 0.3mm after retraction");
}

ZTEST(motion_base, test_pb_move_retraction_limit) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {10.0f, 0, 0};
  
  pb_init(&pb, &src, &dst, true);
  
  // Move forward much more than history size can track
  // EDM_HISTORY_SIZE=201, EDM_RESOLUTION_MM=0.005, so max history ~1mm
  pb_move(&pb, 5.0f);  // Move 5mm forward (way beyond history)
  
  // Try to retract way beyond limit - should fail
  zassert_false(pb_move(&pb, -10.0f), "Retraction beyond history limit should fail");
}

ZTEST(motion_base, test_pb_move_to_end) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {0.5f, 0, 0};  // Short segment
  
  pb_init(&pb, &src, &dst, true);
  
  // Move beyond segment end
  pb_move(&pb, 1.0f);  // Try to move 1mm on 0.5mm segment
  
  zassert_true(pb_at_end(&pb), "Should be at end after overshooting");
  pos_phys_t pos = pb_get_pos(&pb);
  zassert_within(pos.x, 0.5f, EDM_RESOLUTION_MM + 1e-4f, "Should be clipped to segment end");
}

// Test path_buffer_t multi-segment
ZTEST(motion_base, test_pb_write_and_traverse) {
  path_buffer_t pb;
  pos_phys_t p1 = {0, 0, 0};
  pos_phys_t p2 = {1, 0, 0};
  pos_phys_t p3 = {1, 1, 0};  // L-shaped path
  
  pb_init(&pb, &p1, &p2, false);
  
  zassert_true(pb_can_write(&pb), "Should be able to write to non-end segment");
  pb_write(&pb, &p3, true);
  
  // Move through both segments
  pb_move(&pb, 1.5f);  // Should be halfway through second segment
  pos_phys_t pos = pb_get_pos(&pb);
  zassert_within(pos.x, 1.0f, EDM_RESOLUTION_MM + 1e-4f, "X should be at corner");
  zassert_within(pos.y, 0.5f, EDM_RESOLUTION_MM + 1e-4f, "Y should be halfway up");
}

ZTEST(motion_base, test_pb_write_buffer_full) {
  path_buffer_t pb;
  pos_phys_t p1 = {0, 0, 0};
  pos_phys_t p2 = {1, 0, 0};
  pos_phys_t p3 = {2, 0, 0};
  
  pb_init(&pb, &p1, &p2, false);
  pb_write(&pb, &p3, false);  // Fill the buffer
  
  zassert_false(pb_can_write(&pb), "Buffer should be full after one write");
  zassert_true(pb_is_ready(&pb), "If can't write, must be ready (buffer populated)");
  
  // Move to consume the buffered segment
  pb_move(&pb, 1.1f);  // Move past first segment
  
  zassert_true(pb_can_write(&pb), "Should be able to write after consuming buffer");
}

// Test edge cases
ZTEST(motion_base, test_pb_tiny_movements) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {1, 0, 0};
  
  pb_init(&pb, &src, &dst, true);
  
  // Move less than EDM_RESOLUTION_MM - should not change position
  pos_phys_t pos_before = pb_get_pos(&pb);
  pb_move(&pb, EDM_RESOLUTION_MM * 0.5f);
  pos_phys_t pos_after = pb_get_pos(&pb);
  
  zassert_within(pos_before.x, pos_after.x, 1e-4f, "Tiny movement should not change discrete position");
}

ZTEST(motion_base, test_pb_zero_length_segment) {
  path_buffer_t pb;
  pos_phys_t same = {5, 5, 5};
  
  pb_init(&pb, &same, &same, true);  // Zero-length segment
  
  pb_move(&pb, 1.0f);  // Should not crash
  zassert_true(pb_at_end(&pb), "Zero-length segment should be at end");
  
  pos_phys_t pos = pb_get_pos(&pb);
  zassert_within(pos.x, 5.0f, 1e-4f, "Should stay at same position");
}

ZTEST(motion_base, test_pb_accumulated_tiny_movements) {
  path_buffer_t pb;
  pos_phys_t src = {0, 0, 0};
  pos_phys_t dst = {1, 0, 0};
  
  pb_init(&pb, &src, &dst, true);
  
  // Accumulate tiny movements until they add up to one notch
  float tiny = EDM_RESOLUTION_MM * 0.3f;
  pb_move(&pb, tiny);  // 0.3 * 0.005 = 0.0015mm
  pb_move(&pb, tiny);  // 0.6 * 0.005 = 0.003mm  
  pb_move(&pb, tiny);  // 0.9 * 0.005 = 0.0045mm
  pb_move(&pb, tiny);  // 1.2 * 0.005 = 0.006mm -> should trigger one notch
  
  pos_phys_t pos = pb_get_pos(&pb);
  zassert_true(pos.x >= EDM_RESOLUTION_MM - 1e-4f, "Accumulated tiny movements should eventually advance position");
}

ZTEST_SUITE(motion_base, NULL, NULL, NULL, NULL, NULL);
