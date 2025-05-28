// SPDX-License-Identifier: AGPL-3.0-or-later
#include "grbl/edm_base.h"

#include <math.h>
#include <string.h>

float posp_dist(const pos_phys_t* a, const pos_phys_t* b) {
  return sqrtf((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y) +
               (a->z - b->z) * (a->z - b->z));
}

void posp_interp(const pos_phys_t* a,
                 const pos_phys_t* b,
                 float t,
                 pos_phys_t* out) {
  out->x = a->x + (b->x - a->x) * t;
  out->y = a->y + (b->y - a->y) * t;
  out->z = a->z + (b->z - a->z) * t;
}

void pb_init(path_buffer_t* pb,
             const pos_phys_t* src,
             const pos_phys_t* dst,
             bool dst_is_end) {
  memset(pb, 0, sizeof(path_buffer_t));

  pb->pos_history[0] = *src;
  pb->num_history = 1;

  pb->curr_seg_src = *src;
  pb->curr_seg_dst = *dst;
  pb->curr_seg_dst_is_end = dst_is_end;
}

pos_phys_t pb_get_pos(const path_buffer_t* pb) {
  int ix_read = (pb->ix_history + EDM_HISTORY_SIZE - pb->notches_retract) %
                EDM_HISTORY_SIZE;
  return pb->pos_history[ix_read];
}

bool pb_at_end(const path_buffer_t* pb) {
  if (pb->notches_retract > 0) {
    return false;
  }

  if (!pb->curr_seg_dst_is_end) {
    return false;
  }

  // TODO: this is ok, but might not be consistent.
  const pos_phys_t* curr = &pb->pos_history[pb->ix_history];
  return posp_dist(curr, &pb->curr_seg_dst) <= EDM_RESOLUTION_MM;
}

bool pb_can_write(const path_buffer_t* pb) {
  return !pb->curr_seg_dst_is_end && !pb->next_seg_avail;
}

void pb_write(path_buffer_t* pb, const pos_phys_t* next_pos, bool is_end) {
  pb->next_pos = *next_pos;
  pb->next_pos_is_end = is_end;
  pb->next_seg_avail = true;
}

bool pb_is_ready(const path_buffer_t* pb) {
  return pb->curr_seg_dst_is_end || pb->next_seg_avail;
}

static inline int mini(int a, int b) {
  return (a < b) ? a : b;
}

static inline void push_history(path_buffer_t* pb, const pos_phys_t* pos) {
  pb->ix_history = (pb->ix_history + 1) % EDM_HISTORY_SIZE;
  pb->pos_history[pb->ix_history] = *pos;
  if (pb->num_history < EDM_HISTORY_SIZE) {
    pb->num_history++;
  }
}

bool pb_move(path_buffer_t* pb, float d) {
  // dist <-> notch + fraction conversion.
  pb->fraction += d;
  int d_notches = truncf(pb->fraction * (1.0f / EDM_RESOLUTION_MM));
  if (d_notches == 0) {
    // nothing to do.
    return true;
  }
  pb->fraction -= d_notches * EDM_RESOLUTION_MM;

  // Consume d_ticks by moving history.
  if (d_notches < 0) {
    // go back in history as much as possible.
    int available = pb->num_history - pb->notches_retract - 1;  // always >= 0
    if (d_notches < -available) {
      // Retract limit exceeded. Clip to furthest possible with error.
      pb->notches_retract += available;
      return false;
    } else {
      pb->notches_retract += -d_notches;
      return true;
    }
  } else {
    // for forward in history as much as possible.
    if (pb->notches_retract > d_notches) {
      pb->notches_retract -= d_notches;
      return true;
    } else {
      d_notches -= pb->notches_retract;
      pb->notches_retract = 0;
      // need to continue
    }
  }

  // When this point is reached, d_notches > 0, and we need to actually move
  // forward.
  for (int i = 0; i < d_notches; i++) {
    bool clipped = false;
    float seg_len = posp_dist(&pb->curr_seg_src, &pb->curr_seg_dst);

    pb->curr_seg_d += EDM_RESOLUTION_MM;
    if (pb->curr_seg_d >= seg_len) {
      // overflown
      if (pb->curr_seg_dst_is_end || !pb->next_seg_avail) {
        // should not (or cannot) move to next segment.
        pb->curr_seg_d = seg_len;
        clipped = true;
      } else {
        // move to next segment.
        pb->curr_seg_d -= seg_len;
        pb->curr_seg_src = pb->curr_seg_dst;
        pb->curr_seg_dst = pb->next_pos;
        pb->curr_seg_dst_is_end = pb->next_pos_is_end;
        pb->next_seg_avail = false;
      }
    }

    // Record point to history.
    pos_phys_t pos;
    if (seg_len < EDM_RESOLUTION_MM) {
      // avoid division error.
      pos = pb->curr_seg_src;
    } else {
      posp_interp(&pb->curr_seg_src, &pb->curr_seg_dst,
                  pb->curr_seg_d / seg_len, &pos);
    }
    push_history(pb, &pos);

    // If clipped, history nor seg_d will not move further. (not an error)
    if (clipped) {
      break;
    }
  }
  return true;
}
