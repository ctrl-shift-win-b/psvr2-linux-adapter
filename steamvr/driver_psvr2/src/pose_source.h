// SPDX-License-Identifier: GPL-2.0
#pragma once

#include "openvr_driver.h"

// Bridges libpsvr2's 6DoF stream to an OpenVR DriverPose_t. Owns the libpsvr2
// handle and converts the device-native frame into the OpenVR frame.
class PoseSource
{
public:
	PoseSource();
	~PoseSource();

	// True once the /dev/psvr2-pose node was found and opened.
	bool HasPose() const;

	// Block (up to a short timeout) for the next sample and fill `out` in the
	// OpenVR frame. Returns true if `out` was updated, false on timeout/EOF.
	// Fills vecVelocity / vecAngularVelocity from smoothed sample-to-sample
	// deltas so SteamVR's own predictor has something to extrapolate with.
	bool ReadPose( vr::DriverPose_t &out );

	// True if the IMU node is available (enables GetFusedPose).
	bool HasImu() const;

	// Fusion tuning. slam_latency_s: how far behind real time a SLAM sample is
	// when it arrives (camera exposure + compute + transport). tau_s: time
	// constant over which drift corrections are blended in.
	void SetFusionParams( double slam_latency_s, double tau_s );

	// Offset from the tracked origin (IMU, front of visor) to the head/mid-eye
	// origin, in the device-local OpenVR frame (x right, y up, z backward).
	void SetHeadOffset( double x, double y, double z );

	// Accelerometer bias (wire axes, m/s^2, from tools/accel-bias-cal.py) and
	// the position-correction time constant.
	void SetAccelParams( double bias0, double bias1, double bias2, double pos_tau_s );

	// Enable/disable accel dead-reckoning (EXPERIMENTAL). Disabled, translation
	// falls back to critically-damped smoothing of SLAM positions only.
	void SetAccelFusion( bool enable );

	// Full wire->body accel matrix (row-major, from tools/accel-matrix-cal.py);
	// replaces the default signed permutation.
	void SetAccelMatrix( const double m[9] );

	// Non-blocking fused pose at ~500 Hz. The gyro-integrated estimate owns the
	// present; SLAM samples are compared against the buffered estimate at their
	// own capture time and the residual (pure drift) is blended in over tau.
	// SLAM never snaps the displayed pose except on large errors (recovery).
	// Returns false until the first SLAM sample arrives.
	bool GetFusedPose( vr::DriverPose_t &out );

private:
	struct psvr2 *dev_;

	// Fusion state (orientation kept in the raw device frame).
	bool have_base_ = false;
	bool base_valid_ = false;
	vr::HmdQuaternion_t est_q_raw_{ 1.0, 0.0, 0.0, 0.0 };
	vr::HmdQuaternion_t err_pending_{ 1.0, 0.0, 0.0, 0.0 }; // world-side drift residual
	uint64_t last_fuse_host_ns_ = 0;

	// Translational state, raw world frame: accel dead-reckoning corrected
	// against timestamp-matched SLAM positions (complementary, critically
	// damped: the position error feeds both position and velocity).
	double p_raw_[3] = { 0.0, 0.0, 0.0 };
	double v_raw_[3] = { 0.0, 0.0, 0.0 };
	// World-frame acceleration bias integrator (3rd filter order): absorbs
	// residual sensor bias and any steady inertial disturbance the room-fixed
	// SLAM never sees (e.g. use on a vehicle or vessel).
	double ab_raw_[3] = { 0.0, 0.0, 0.0 };
	double pos_err_pending_[3] = { 0.0, 0.0, 0.0 };
	uint64_t last_slam_host_ns_ = 0;

	double slam_latency_s_ = 0.020;
	double tau_s_ = 0.2;
	double pos_tau_s_ = 0.2;
	// Wire-axis accel bias, m/s^2 (measured 2026-07-10, tools/accel-bias-cal.py).
	double accel_bias_[3] = { -0.0114, 0.0085, 0.3138 };
	// Wire->body accel matrix. Same MEMS die as the gyro, so this is exactly
	// the NEGATED full gyro rotation (pseudovector vs vector under the det=-1
	// basis mapping) — both independent calibrations agree on this. Overridable
	// via SetAccelMatrix.
	double accel_mat_[3][3] = {
		{ +0.017199, +0.001900, +0.999850 },
		{ -0.005711, -0.999982, +0.001998 },
		{ +0.999836, -0.005744, -0.017188 },
	};
	// EXPERIMENTAL, default off: accel dead-reckoning shows lateral coupling
	// (azimuthal accel-frame misalignment the static cal can't observe; the
	// dynamic full-matrix cal needs a better-conditioned formulation — see
	// docs/PHASE2_PSVR2.md). Off = critically-damped SLAM smoothing (tau 60 ms).
	bool accel_fusion_ = false;

	// IMU -> mid-eye offset (Monado T_imu_head; the eyes sit ~10.5 cm behind
	// the front-mounted IMU). Rendering at the IMU instead exaggerates
	// close-range parallax during head rotation.
	double head_offset_[3] = { 0.000247, -0.000273, 0.104826 };

	// Ring buffer of timestamped orientation estimates (~1 s at 500 Hz) so a
	// late SLAM sample can be compared against its contemporary estimate.
	static constexpr int kHistSize = 512;
	struct HistEntry
	{
		uint64_t t_ns;
		vr::HmdQuaternion_t q;
		double p[3];
	};
	HistEntry hist_[kHistSize] = {};
	int hist_head_ = 0;
	int hist_count_ = 0;

	// Previous remapped estimate for high-rate angular velocity.
	bool prev_est_valid_ = false;
	double prev_est_t_s_ = 0.0;
	vr::HmdQuaternion_t prev_est_q_{ 1.0, 0.0, 0.0, 0.0 };

	// Previous remapped sample for finite-difference velocities.
	bool prev_valid_ = false;
	double prev_ts_s_ = 0.0;
	double prev_pos_[3] = { 0.0, 0.0, 0.0 };
	vr::HmdQuaternion_t prev_q_{ 1.0, 0.0, 0.0, 0.0 };

	// Exponentially smoothed velocities (world/driver frame).
	double vel_s_[3] = { 0.0, 0.0, 0.0 };
	double angvel_s_[3] = { 0.0, 0.0, 0.0 };

	// One-shot stream-rate measurement (logged ~5 s after the first sample).
	double rate_t0_s_ = 0.0;
	unsigned rate_count_ = 0;
	double rate_dt_min_ = 1e9, rate_dt_max_ = 0.0;
	bool rate_logged_ = false;
};
