// SPDX-License-Identifier: GPL-2.0
// Shim header for psvr2_distortion.c (ported from Monado's psvr2-work-steamvr
// branch, BSL-1.0, Joel Valenciano / Beyley Cardellio). Replaces Monado's
// <xrt/xrt_defines.h> with the two types the port needs. Layout must remain
// three consecutive float pairs — the port indexes it as xrt_vec2[3].
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

struct xrt_vec2
{
	float x, y;
};

struct xrt_uv_triplet
{
	struct xrt_vec2 r, g, b;
};

// calibration: 8 floats — per-eye X offsets [0]/[2], Y offsets [1]/[3],
// tilt rotation cos/sin left [4]/[5], right [6]/[7]. eEye: 0 = left.
// fU/fV: per-eye UV in [0,1]. outCoords: per-channel distorted UVs.
void psvr2_compute_distortion_asymmetric(float *calibration, struct xrt_uv_triplet *outCoords, int eEye, float fU,
                                         float fV);

#ifdef __cplusplus
}
#endif
