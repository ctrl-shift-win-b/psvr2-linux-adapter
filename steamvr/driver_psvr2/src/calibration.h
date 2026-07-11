// SPDX-License-Identifier: GPL-2.0
#pragma once

#include <cstddef>

// Reads the headset's factory distortion calibration (vendor control report
// 0x8f) and remaps it into the 8-float array psvr2_compute_distortion_asymmetric
// expects. On any failure `out` holds generic fallback values (identity tilt,
// nominal offsets) and false is returned; `detail` receives a one-line
// human-readable outcome either way.
bool Psvr2ReadFactoryCalibration( float out[8], char *detail, size_t detail_len );
