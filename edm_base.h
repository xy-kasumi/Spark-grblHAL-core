// SPDX-License-Identifier: AGPL-3.0-or-later
//
// This file defines pure computational data structures w/o any side effects.
#pragma once

#include <stdbool.h>

// Positional resolution of EDM control in mm.
// Internally, everything is handled by "notch" of this length along motion
// path.
#define EDM_RESOLUTION_MM 0.005f

// EDM_RESOLUTION_MM * (EDM_HISTORY_SIZE - 1) will be the maximum retractable
// distance. e.g. if EDM_RESOLUTION_MM = 0.005, EDM_HISTORY_SIZE = 201, then
// retraction up to 1.0 mm is possible.
#define EDM_HISTORY_SIZE 201

// Represents a single physical coordinate. (i.e. coordinates specification in
// G-code)
typedef struct {
  float x;
  float y;
  float z;
  // a, b, c etc.
} pos_phys_t;

// Compute distance between two pos_phys_t points.
// Returns distance in mm.
// TODO: What to do about rotational axes?
float posp_dist(const pos_phys_t* a, const pos_phys_t* b);

// Linearly interpolate between a (t=0), and b (t=1).
// t can be outside of [0, 1] range, in which case it will be extrapolated.
void posp_interp(const pos_phys_t* a,
                 const pos_phys_t* b,
                 float t,
                 pos_phys_t* out);

// path_buffer_t represents a path and a current position, typed by pos_phys_t.
//
// The path is a sequence of line segments. It can be extended continuously
// while being used, until its end is marked.
//
// Current position is controlled by pb_move(), both forward and backward motion
// is allowed. path_buffer_t guarantees that the position is always on the
// path and will not go out of bounds.
//
// Furthest traveled position is also tracked.
// pb_move() reports error if it tries to go back beyond maximum retractable
// distance (see EDM_HISTORY_SIZE).
typedef struct {
  // Ring buffer. History is recorded at EDM_RESOLUTION_MM "notches".
  pos_phys_t pos_history[EDM_HISTORY_SIZE];
  // index of current position in pos_history.
  int ix_history;
  // number of valid elements in pos_history. always in [1, EDM_HISTORY_SIZE]
  int num_history;

  // 0: current position is furthest (curr_seg_d).
  // positive: notches_retract==1: 1 previous tick in history, and so on.
  // [0, num_history - 1] is allowed.
  int notches_retract;

  // Point on the segment corresponding to notches_retract == 0.
  float curr_seg_d;
  pos_phys_t curr_seg_src;
  pos_phys_t curr_seg_dst;
  bool curr_seg_dst_is_end;

  // single-element buffer for writing next segment.
  bool next_seg_avail;
  pos_phys_t next_pos;
  bool next_pos_is_end;

  // internal_pos (notch-aligned) + fraction = current pb_move() position.
  // always |fraction| < EDM_RESOLUTION_MM
  float fraction;
} path_buffer_t;

// This must be called before any other path_buffer_t calls.
// Initialize with single line segment.
void pb_init(path_buffer_t* pb,
             const pos_phys_t* src,
             const pos_phys_t* dst,
             bool dst_is_end);

// Get the current (notch-aligned) position.
pos_phys_t pb_get_pos(const path_buffer_t* pb);

// Get if the current position is at the end of the path.
bool pb_at_end(const path_buffer_t* pb);

// Fetches whether path buffer has a room for pb_write() calls.
bool pb_can_write(const path_buffer_t* pb);

// Write next point of path.
// Should only be called when pb_can_write() is true.
// (Otherwise, previous writes might get overwritten.)
//
// If is_end is false, further pb_write is allowed.
// If is_end is true, next_pos will be the end of the path, and no further
// pb_write is allowed.
void pb_write(path_buffer_t* pb, const pos_phys_t* next_pos, bool is_end);

// Fetches whether the path buffer is ready for pb_move() calls.
bool pb_is_ready(const path_buffer_t* pb);

// Move current position along the path by distance d.
// d can be positive (forward), or negative (backward).
//
// Actual position is discretized by EDM_RESOLUTION_MM notches,
// so tiny moves will not appear immediately in pb_get_pos().
//
// If the resulting position is out of bounds of current path or retraction
// limit, it will be clipped.
//
// Returns true if ok. Returns false iff max retraction was exceeded.
bool pb_move(path_buffer_t* pb, float d);
