// SPDX-License-Identifier: GPL-2.0
#include "pose_source.h"

#include <cmath>
#include <ctime>

#include "driverlog.h"
#include "libpsvr2.h"

// ---------------------------------------------------------------------------
// Device -> OpenVR frame remap.
//
// libpsvr2 reports the headset's native raw frame (see kernel/psvr2_uapi.h):
//   position[0]: forward,  position[1]: up,  position[2]: right
//   orientation: quaternion (w, x, y, z) in that same raw frame
//
// OpenVR is right-handed: +x right, +y up, -z forward, metres. OpenXR uses the
// same convention, so we mirror the Monado PSVR2 driver's process_slam_record
// remap verbatim (see docs/references.md) — adopted as the reference after the
// in-headset pose-guide superseded the earlier hand-timed capture.
//
// The tracker's native upright is additionally rolled 90deg from OpenVR's, so
// after the basis change we left-multiply a fixed +90deg rotation about Z (the
// forward axis) — Monado's SLAM_POSE_CORRECTION. Confirmed on hardware: without
// it the rendered view sits 90deg rolled to the left.
// ---------------------------------------------------------------------------

// Hamilton product r = a * b, both/all (w, x, y, z).
static vr::HmdQuaternion_t QuatMul( const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b )
{
	return {
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	};
}

static void RemapPose( const struct psvr2_pose &in, vr::DriverPose_t &out )
{
	// Position: raw (forward, up, right) -> OpenVR (right, up, -forward).
	out.vecPosition[0] = in.position[2];
	out.vecPosition[1] = in.position[1];
	out.vecPosition[2] = -in.position[0];

	// Orientation: basis change on the quaternion (matches process_slam_record).
	const vr::HmdQuaternion_t remapped = {
		in.orientation[0],   // w
		-in.orientation[2],  // x
		-in.orientation[1],  // y
		in.orientation[3],   // z
	};

	// Fixed +90deg roll about Z (sqrt(2)/2 ~= cos/sin 45deg), left-multiplied.
	static const double s = 0.70710678118654752;
	static const vr::HmdQuaternion_t kRollCorrection = { s, 0.0, 0.0, s };
	out.qRotation = QuatMul( kRollCorrection, remapped );
}

// Same basis change + roll correction as RemapPose, for a quaternion kept in
// the raw device frame (w = orientation[0], x/y/z = orientation[1..3]).
static vr::HmdQuaternion_t RemapRawQuat( const vr::HmdQuaternion_t &raw )
{
	const vr::HmdQuaternion_t remapped = { raw.w, -raw.y, -raw.x, raw.z };
	static const double s = 0.70710678118654752;
	static const vr::HmdQuaternion_t kRollCorrection = { s, 0.0, 0.0, s };
	return QuatMul( kRollCorrection, remapped );
}

// World-frame angular rate (rad/s, axis components) from two orientations.
static void AngularRate( const vr::HmdQuaternion_t &q_prev, const vr::HmdQuaternion_t &q_now, double dt,
                         double w_out[3] )
{
	const vr::HmdQuaternion_t conj_prev = { q_prev.w, -q_prev.x, -q_prev.y, -q_prev.z };
	vr::HmdQuaternion_t dq = QuatMul( q_now, conj_prev );
	if ( dq.w < 0.0 )
	{
		dq.w = -dq.w; dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z;
	}
	const double sin_half = std::sqrt( dq.x * dq.x + dq.y * dq.y + dq.z * dq.z );
	if ( sin_half > 1e-9 )
	{
		const double rate = 2.0 * std::atan2( sin_half, dq.w ) / dt / sin_half;
		w_out[0] = dq.x * rate;
		w_out[1] = dq.y * rate;
		w_out[2] = dq.z * rate;
	}
	else
	{
		w_out[0] = w_out[1] = w_out[2] = 0.0;
	}
}

static uint64_t MonotonicNs()
{
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	return static_cast<uint64_t>( ts.tv_sec ) * 1000000000ull + ts.tv_nsec;
}

static vr::HmdQuaternion_t QuatConj( const vr::HmdQuaternion_t &q )
{
	return { q.w, -q.x, -q.y, -q.z };
}

// q^f for f in [0,1]: fraction of the rotation, shortest arc.
static vr::HmdQuaternion_t QuatPow( const vr::HmdQuaternion_t &q_in, double f )
{
	vr::HmdQuaternion_t q = q_in;
	if ( q.w < 0.0 )
	{
		q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z;
	}
	const double sin_half = std::sqrt( q.x * q.x + q.y * q.y + q.z * q.z );
	if ( sin_half < 1e-12 )
		return { 1.0, 0.0, 0.0, 0.0 };
	const double half = std::atan2( sin_half, q.w );
	const double s = std::sin( half * f ) / sin_half;
	return { std::cos( half * f ), q.x * s, q.y * s, q.z * s };
}

static double QuatAngle( const vr::HmdQuaternion_t &q )
{
	const double sin_half = std::sqrt( q.x * q.x + q.y * q.y + q.z * q.z );
	return 2.0 * std::atan2( sin_half, std::fabs( q.w ) );
}

// Rotate vector v by quaternion q: out = q v q^-1.
static void QuatRotateVec( const vr::HmdQuaternion_t &q, const double v[3], double out[3] )
{
	const double tx = 2.0 * ( q.y * v[2] - q.z * v[1] );
	const double ty = 2.0 * ( q.z * v[0] - q.x * v[2] );
	const double tz = 2.0 * ( q.x * v[1] - q.y * v[0] );
	out[0] = v[0] + q.w * tx + q.y * tz - q.z * ty;
	out[1] = v[1] + q.w * ty + q.z * tx - q.x * tz;
	out[2] = v[2] + q.w * tz + q.x * ty - q.y * tx;
}

PoseSource::PoseSource() : dev_( psvr2_open() ) {}

PoseSource::~PoseSource()
{
	if ( dev_ )
		psvr2_close( dev_ );
}

bool PoseSource::HasPose() const
{
	return dev_ && psvr2_has_pose( dev_ );
}

bool PoseSource::ReadPose( vr::DriverPose_t &out )
{
	struct psvr2_pose sample{};
	if ( !dev_ || psvr2_read_pose( dev_, &sample, /*block=*/1 ) != 1 )
		return false;

	out = vr::DriverPose_t{};

	// Identity offsets: the device already reports head pose in world space.
	out.qWorldFromDriverRotation.w = 1.0;
	out.qDriverFromHeadRotation.w = 1.0;
	out.vecDriverFromHeadTranslation[0] = head_offset_[0];
	out.vecDriverFromHeadTranslation[1] = head_offset_[1];
	out.vecDriverFromHeadTranslation[2] = head_offset_[2];

	RemapPose( sample, out );

	out.poseIsValid = sample.valid != 0;
	out.deviceIsConnected = true;
	out.result = sample.valid ? vr::TrackingResult_Running_OK
	                          : vr::TrackingResult_Running_OutOfRange;
	out.shouldApplyHeadModel = false; // we provide a real 6DoF pose

	// ---- Velocities (finite differences on the remapped pose) --------------
	// SteamVR's pose predictor extrapolates with the velocities the driver
	// supplies; leaving them zero disables prediction entirely and every ms of
	// pipeline latency shows up as head-drag. EMA smoothing damps SLAM jitter
	// (the predictor multiplies velocity noise by the prediction horizon).
	const double now_s = sample.timestamp_ns * 1e-9;
	if ( prev_valid_ && sample.valid )
	{
		const double dt = now_s - prev_ts_s_;
		if ( dt > 1e-4 && dt < 0.25 )
		{
			constexpr double kAlpha = 0.4;

			for ( int i = 0; i < 3; i++ )
			{
				const double v = ( out.vecPosition[i] - prev_pos_[i] ) / dt;
				vel_s_[i] = kAlpha * v + ( 1.0 - kAlpha ) * vel_s_[i];
			}

			// World-frame delta rotation dq = q_now * conj(q_prev), shortest arc,
			// as an axis-angle rate.
			const vr::HmdQuaternion_t conj_prev = { prev_q_.w, -prev_q_.x, -prev_q_.y, -prev_q_.z };
			vr::HmdQuaternion_t dq = QuatMul( out.qRotation, conj_prev );
			if ( dq.w < 0.0 )
			{
				dq.w = -dq.w; dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z;
			}
			const double sin_half = std::sqrt( dq.x * dq.x + dq.y * dq.y + dq.z * dq.z );
			if ( sin_half > 1e-9 )
			{
				const double angle = 2.0 * std::atan2( sin_half, dq.w );
				const double rate = angle / dt / sin_half; // rad/s per axis component
				const double w[3] = { dq.x * rate, dq.y * rate, dq.z * rate };
				for ( int i = 0; i < 3; i++ )
					angvel_s_[i] = kAlpha * w[i] + ( 1.0 - kAlpha ) * angvel_s_[i];
			}
			else
			{
				for ( int i = 0; i < 3; i++ )
					angvel_s_[i] *= ( 1.0 - kAlpha );
			}

			// One-shot rate report (B1 measurement).
			if ( !rate_logged_ )
			{
				if ( rate_count_ == 0 )
					rate_t0_s_ = now_s;
				rate_count_++;
				rate_dt_min_ = std::fmin( rate_dt_min_, dt );
				rate_dt_max_ = std::fmax( rate_dt_max_, dt );
				const double elapsed = now_s - rate_t0_s_;
				if ( elapsed >= 5.0 )
				{
					DriverLog( "psvr2: pose stream %.1f Hz over %.1f s (dt min %.1f ms / max %.1f ms)",
					           rate_count_ / elapsed, elapsed, rate_dt_min_ * 1e3, rate_dt_max_ * 1e3 );
					rate_logged_ = true;
				}
			}
		}
	}

	for ( int i = 0; i < 3; i++ )
	{
		out.vecVelocity[i] = vel_s_[i];
		out.vecAngularVelocity[i] = angvel_s_[i];
	}

	prev_valid_ = sample.valid != 0;
	prev_ts_s_ = now_s;
	for ( int i = 0; i < 3; i++ )
		prev_pos_[i] = out.vecPosition[i];
	prev_q_ = out.qRotation;

	return true;
}

bool PoseSource::HasImu() const
{
	return dev_ && psvr2_has_imu( dev_ );
}

void PoseSource::SetFusionParams( double slam_latency_s, double tau_s )
{
	if ( slam_latency_s >= 0.0 && slam_latency_s < 0.5 )
		slam_latency_s_ = slam_latency_s;
	if ( tau_s > 0.01 && tau_s < 5.0 )
		tau_s_ = tau_s;
}

void PoseSource::SetHeadOffset( double x, double y, double z )
{
	head_offset_[0] = x;
	head_offset_[1] = y;
	head_offset_[2] = z;
}

void PoseSource::SetAccelParams( double bias0, double bias1, double bias2, double pos_tau_s )
{
	accel_bias_[0] = bias0;
	accel_bias_[1] = bias1;
	accel_bias_[2] = bias2;
	if ( pos_tau_s > 0.01 && pos_tau_s < 5.0 )
		pos_tau_s_ = pos_tau_s;
}

void PoseSource::SetAccelFusion( bool enable )
{
	accel_fusion_ = enable;
}

void PoseSource::SetAccelMatrix( const double m[9] )
{
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
			accel_mat_[i][j] = m[i * 3 + j];
}

bool PoseSource::GetFusedPose( vr::DriverPose_t &out )
{
	if ( !dev_ )
		return false;

	// Drain pending SLAM samples. SLAM never touches the displayed estimate
	// directly: each sample is compared against the buffered estimate at its
	// own capture time (receive time minus the configured latency); the
	// residual is drift and is queued to be blended in over tau.
	struct psvr2_pose s {};
	while ( psvr2_read_pose( dev_, &s, /*block=*/0 ) == 1 )
	{
		const double now_s = s.timestamp_ns * 1e-9;
		if ( s.valid )
		{
			const double pos_ovr[3] = { s.position[2], s.position[1], -s.position[0] };

			if ( prev_valid_ )
			{
				const double dt = now_s - prev_ts_s_;
				if ( dt > 1e-4 && dt < 0.25 )
				{
					constexpr double kAlphaLin = 0.4;
					for ( int i = 0; i < 3; i++ )
					{
						const double v = ( pos_ovr[i] - prev_pos_[i] ) / dt;
						vel_s_[i] = kAlphaLin * v + ( 1.0 - kAlphaLin ) * vel_s_[i];
					}

					if ( !rate_logged_ )
					{
						if ( rate_count_ == 0 )
							rate_t0_s_ = now_s;
						rate_count_++;
						rate_dt_min_ = std::fmin( rate_dt_min_, dt );
						rate_dt_max_ = std::fmax( rate_dt_max_, dt );
						const double elapsed = now_s - rate_t0_s_;
						if ( elapsed >= 5.0 )
						{
							DriverLog( "psvr2: SLAM stream %.1f Hz (dt %.1f-%.1f ms), error-state gyro fusion "
							           "(latency %.0f ms, tau %.0f ms)",
							           rate_count_ / elapsed, rate_dt_min_ * 1e3, rate_dt_max_ * 1e3,
							           slam_latency_s_ * 1e3, tau_s_ * 1e3 );
							rate_logged_ = true;
						}
					}
				}
			}
			prev_valid_ = true;
			prev_ts_s_ = now_s;
			for ( int i = 0; i < 3; i++ )
				prev_pos_[i] = pos_ovr[i];

			const vr::HmdQuaternion_t slam_q = { s.orientation[0], s.orientation[1], s.orientation[2],
			                                     s.orientation[3] };

			if ( !have_base_ )
			{
				// First fix: adopt outright.
				est_q_raw_ = slam_q;
				for ( int i = 0; i < 3; i++ )
				{
					p_raw_[i] = s.position[i];
					v_raw_[i] = 0.0;
				}
				have_base_ = true;
			}
			else
			{
				// Estimate at the sample's capture time, from the ring buffer.
				const uint64_t t_ref =
				    s.timestamp_ns - static_cast<uint64_t>( slam_latency_s_ * 1e9 );
				vr::HmdQuaternion_t q_then = est_q_raw_;
				const double *p_then = p_raw_;
				for ( int i = 0, idx = hist_head_; i < hist_count_; i++ )
				{
					idx = ( idx + kHistSize - 1 ) % kHistSize;
					q_then = hist_[idx].q;
					p_then = hist_[idx].p;
					if ( hist_[idx].t_ns <= t_ref )
						break;
				}

				const vr::HmdQuaternion_t err = QuatMul( slam_q, QuatConj( q_then ) );
				if ( QuatAngle( err ) > 0.35 ) // ~20 deg: tracking recovery, snap
				{
					est_q_raw_ = slam_q;
					err_pending_ = { 1.0, 0.0, 0.0, 0.0 };
				}
				else
				{
					// Merge into the pending residual (newest measurement wins the
					// bulk; residuals are tiny so ordering hardly matters).
					err_pending_ = QuatMul( err, QuatPow( err_pending_, 0.5 ) );
				}

				// Position residual at matched time.
				double e[3];
				double e2 = 0.0;
				for ( int i = 0; i < 3; i++ )
				{
					e[i] = s.position[i] - p_then[i];
					e2 += e[i] * e[i];
				}
				if ( e2 > 0.25 ) // >0.5 m: tracking recovery, snap ALL translation state
				{
					for ( int i = 0; i < 3; i++ )
					{
						p_raw_[i] = s.position[i];
						v_raw_[i] = 0.0;
						ab_raw_[i] = 0.0;
						pos_err_pending_[i] = 0.0;
					}
				}
				else
				{
					// alpha-beta-gamma tracking corrections, applied per
					// measurement with FRACTIONAL gains: the accelerometer owns
					// motion transients; SLAM only steers the running mean.
					// (Full-gain correction re-imposes SLAM's lagged onsets and
					// reduces the fusion to interpolated SLAM.)
					// Tight coupling: dead-reckoning velocity accumulates error
					// during sustained motion (soft gains let it linger as
					// overshoot), so corrections run strong — alpha 0.5 with
					// near-optimal beta (alpha^2/(2-alpha)).
					constexpr double kA = 0.5;   // position fraction
					constexpr double kB = 0.15;  // velocity fraction (x rate)
					constexpr double kG = 0.002; // accel-bias fraction (x rate^2)
					constexpr double kRate = 30.0; // nominal SLAM rate, 1/s
					for ( int i = 0; i < 3; i++ )
					{
						p_raw_[i] += kA * e[i];
						v_raw_[i] += kB * kRate * e[i];
						ab_raw_[i] -= kG * kRate * kRate * e[i] * ( accel_fusion_ ? 1.0 : 0.0 );
					}
				}
			}

			last_slam_host_ns_ = s.timestamp_ns;
			base_valid_ = true;
		}
		else
		{
			base_valid_ = false;
		}
	}

	if ( !have_base_ )
		return false;

	// Integrate the latest gyro sample over the wall-clock step since the last
	// call. Rates are body-frame in the same raw device basis as the SLAM
	// orientation, so this is a right-multiplication — no axis remap needed.
	const uint64_t now_ns = MonotonicNs();
	double loop_dt = 0.0;
	struct psvr2_imu imu {};
	if ( last_fuse_host_ns_ != 0 && psvr2_read_imu( dev_, &imu ) == 0 )
	{
		loop_dt = ( now_ns - last_fuse_host_ns_ ) * 1e-9;
		if ( loop_dt > 0.0 && loop_dt < 0.05 )
		{
			// Wire gyro -> SLAM quaternion frame: FULL measured rotation from
			// tools/gyro-axis-cal.py (2026-07-10), Gram-Schmidt-orthogonalized
			// preserving det=-1 (pseudovector mapping). The off-diagonal ~1 deg
			// terms are the physical chip-vs-camera mounting misalignment.
			// SLAM quats use the inverse (world-side-delta) convention, so the
			// increment is LEFT-multiplied.
			static const double kGyroMat[3][3] = {
				{ -0.017199, -0.001900, -0.999850 },
				{ +0.005711, +0.999982, -0.001998 },
				{ -0.999836, +0.005744, +0.017188 },
			};
			const float *g = imu.gyro_rad_s;
			double wv[3];
			for ( int i = 0; i < 3; i++ )
				wv[i] = kGyroMat[i][0] * g[0] + kGyroMat[i][1] * g[1] + kGyroMat[i][2] * g[2];
			const double wx = wv[0], wy = wv[1], wz = wv[2];
			const double mag = std::sqrt( wx * wx + wy * wy + wz * wz );
			if ( mag * loop_dt > 1e-9 )
			{
				const double half = 0.5 * mag * loop_dt;
				const double k = std::sin( half ) / mag;
				const vr::HmdQuaternion_t dq = { std::cos( half ), wx * k, wy * k, wz * k };
				est_q_raw_ = QuatMul( dq, est_q_raw_ );
			}

			// Accelerometer dead-reckoning. Wire accel -> SLAM body frame is the
			// gyro permutation with globally flipped signs (measured 2026-07-10,
			// tools/accel-bias-cal.py, rms 0.031 m/s^2 — the sign flip is the
			// pseudovector/vector distinction under this basis change). rot(q)
			// maps world->body (verified by the same calibration), so body->world
			// uses the conjugate. Raw-world up is axis 1.
			if ( ( now_ns - last_slam_host_ns_ ) * 1e-9 < 0.5 )
			{
				const double aw[3] = { imu.accel_m_s2[0] - accel_bias_[0],
				                       imu.accel_m_s2[1] - accel_bias_[1],
				                       imu.accel_m_s2[2] - accel_bias_[2] };
				double a_body[3];
				for ( int i = 0; i < 3; i++ )
					a_body[i] = accel_mat_[i][0] * aw[0] + accel_mat_[i][1] * aw[1] +
					            accel_mat_[i][2] * aw[2];
				double a_world[3];
				QuatRotateVec( QuatConj( est_q_raw_ ), a_body, a_world );
				a_world[1] -= 9.80665; // gravity reaction, raw-world up = axis 1
				if ( !accel_fusion_ )
					a_world[0] = a_world[1] = a_world[2] = 0.0; // SLAM-only fallback

				// Third-order complementary tracking loop: the SLAM position
				// residual feeds position, velocity, and a world-frame accel-bias
				// integrator. The bias state rejects constant disturbances —
				// residual sensor bias and steady platform acceleration — which a
				// 2nd-order loop turns into a persistent drag during motion.
				// Pure dead-reckoning between SLAM measurements; all corrections
				// happen fractionally at measurement time (alpha-beta-gamma).
				for ( int i = 0; i < 3; i++ )
				{
					v_raw_[i] += ( a_world[i] - ab_raw_[i] ) * loop_dt;
					p_raw_[i] += v_raw_[i] * loop_dt;
				}
			}
			else
			{
				// SLAM silent too long: freeze dead-reckoning so the pose can't
				// fly away on uncorrected accel drift.
				for ( int i = 0; i < 3; i++ )
					v_raw_[i] = 0.0;
			}
		}
		else
		{
			loop_dt = 0.0;
		}
	}
	last_fuse_host_ns_ = now_ns;

	// Blend a slice of the pending drift residual into the estimate. The slice
	// scales with the loop step so tau is honoured regardless of call rate.
	if ( loop_dt > 0.0 && QuatAngle( err_pending_ ) > 1e-7 )
	{
		const double f = std::fmin( loop_dt / tau_s_, 1.0 );
		est_q_raw_ = QuatMul( QuatPow( err_pending_, f ), est_q_raw_ );
		err_pending_ = QuatPow( err_pending_, 1.0 - f );
	}

	// Renormalize (integration + corrections accumulate rounding).
	{
		const double n = std::sqrt( est_q_raw_.w * est_q_raw_.w + est_q_raw_.x * est_q_raw_.x +
		                            est_q_raw_.y * est_q_raw_.y + est_q_raw_.z * est_q_raw_.z );
		if ( n > 1e-9 )
		{
			est_q_raw_.w /= n; est_q_raw_.x /= n; est_q_raw_.y /= n; est_q_raw_.z /= n;
		}
	}

	// Record the estimate so late SLAM samples can be compared at their own time.
	hist_[hist_head_] = { now_ns, est_q_raw_, { p_raw_[0], p_raw_[1], p_raw_[2] } };
	hist_head_ = ( hist_head_ + 1 ) % kHistSize;
	if ( hist_count_ < kHistSize )
		hist_count_++;

	out = vr::DriverPose_t{};
	out.qWorldFromDriverRotation.w = 1.0;
	out.qDriverFromHeadRotation.w = 1.0;
	out.vecDriverFromHeadTranslation[0] = head_offset_[0];
	out.vecDriverFromHeadTranslation[1] = head_offset_[1];
	out.vecDriverFromHeadTranslation[2] = head_offset_[2];

	// Position: accel dead-reckoned, SLAM-corrected estimate (raw -> OpenVR).
	out.vecPosition[0] = p_raw_[2];
	out.vecPosition[1] = p_raw_[1];
	out.vecPosition[2] = -p_raw_[0];
	out.qRotation = RemapRawQuat( est_q_raw_ );
	// SLAM healthy: full 6DoF. SLAM stale/invalid: degrade to 3DoF — gyro
	// orientation keeps running, position holds at the last good estimate,
	// and the pose stays valid so apps keep rendering (rotation-only status
	// lets them show their limited-tracking hint). Recovery is handled by the
	// error-state corrections (with snap thresholds for large residuals).
	const bool slam_fresh = base_valid_ && ( now_ns - last_slam_host_ns_ ) * 1e-9 < 0.5;
	out.poseIsValid = true;
	out.deviceIsConnected = true;
	out.result = slam_fresh ? vr::TrackingResult_Running_OK : vr::TrackingResult_Fallback_RotationOnly;
	out.shouldApplyHeadModel = false;

	// High-rate angular velocity from the gyro-driven estimate. Short EMA —
	// the source is already smooth, we just take the step edges off.
	const double now_s = now_ns * 1e-9;
	if ( prev_est_valid_ )
	{
		const double dt = now_s - prev_est_t_s_;
		if ( dt > 1e-5 && dt < 0.05 )
		{
			double w[3];
			AngularRate( prev_est_q_, out.qRotation, dt, w );
			constexpr double kAlphaAng = 0.25;
			for ( int i = 0; i < 3; i++ )
				angvel_s_[i] = kAlphaAng * w[i] + ( 1.0 - kAlphaAng ) * angvel_s_[i];
		}
	}
	prev_est_valid_ = true;
	prev_est_t_s_ = now_s;
	prev_est_q_ = out.qRotation;

	// Linear velocity from the fusion state (raw world -> OpenVR).
	out.vecVelocity[0] = v_raw_[2];
	out.vecVelocity[1] = v_raw_[1];
	out.vecVelocity[2] = -v_raw_[0];
	for ( int i = 0; i < 3; i++ )
		out.vecAngularVelocity[i] = angvel_s_[i];

	return true;
}
