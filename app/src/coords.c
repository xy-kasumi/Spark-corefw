// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "coords.h"

#include <math.h>

pos_phys_t coords_to_machine(const pos_phys_t* pos,
                             coord_system_t from_cs,
                             const coord_offsets_t* offsets) {
  pos_phys_t result = *pos;

  switch (from_cs) {
    case COORD_SYSTEM_MACHINE:
      // Already in machine coordinates
      break;

    case COORD_SYSTEM_GRINDER:
      // Convert from grinder to machine: add grinder origin
      result.x += offsets->grinder_origin.x;
      result.y += offsets->grinder_origin.y;
      result.z += offsets->grinder_origin.z;
      break;

    case COORD_SYSTEM_WORK:
      // Convert from work to machine: add work origin
      result.x += offsets->work_origin.x;
      result.y += offsets->work_origin.y;
      result.z += offsets->work_origin.z;
      break;
  }

  return result;
}

pos_phys_t coords_from_machine(const pos_phys_t* machine_pos,
                               coord_system_t to_cs,
                               const coord_offsets_t* offsets) {
  pos_phys_t result = *machine_pos;

  switch (to_cs) {
    case COORD_SYSTEM_MACHINE:
      // Already in machine coordinates
      break;

    case COORD_SYSTEM_GRINDER:
      // Convert from machine to grinder: subtract grinder origin
      result.x -= offsets->grinder_origin.x;
      result.y -= offsets->grinder_origin.y;
      result.z -= offsets->grinder_origin.z;
      break;

    case COORD_SYSTEM_WORK:
      // Convert from machine to work: subtract work origin
      result.x -= offsets->work_origin.x;
      result.y -= offsets->work_origin.y;
      result.z -= offsets->work_origin.z;
      break;
  }

  return result;
}

float posp_dist(const pos_phys_t* a, const pos_phys_t* b) {
  float dx = b->x - a->x;
  float dy = b->y - a->y;
  float dz = b->z - a->z;
  return sqrtf(dx * dx + dy * dy + dz * dz);
}

void posp_interp(const pos_phys_t* a,
                 const pos_phys_t* b,
                 float t,
                 pos_phys_t* out) {
  out->x = a->x + (b->x - a->x) * t;
  out->y = a->y + (b->y - a->y) * t;
  out->z = a->z + (b->z - a->z) * t;
}
