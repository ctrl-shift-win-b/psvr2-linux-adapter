// SPDX-License-Identifier: GPL-2.0
#include "hmd_device_driver.h"

#include <chrono>
#include <cmath>
#include <cstring>

#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>

#include "calibration.h"
#include "driverlog.h"
#include "psvr2_distortion.h"

// The kernel module reports the IPD dial as ABS_MISC (millimetres) on the
// "PlayStation VR2 Headset Controls" input device. Returns an open fd or -1.
static int OpenIpdInputDevice()
{
	DIR *dir = opendir( "/dev/input" );
	if ( !dir )
		return -1;
	int fd = -1;
	struct dirent *de;
	while ( fd < 0 && ( de = readdir( dir ) ) )
	{
		if ( strncmp( de->d_name, "event", 5 ) != 0 )
			continue;
		char path[64];
		snprintf( path, sizeof( path ), "/dev/input/%s", de->d_name );
		const int f = open( path, O_RDONLY | O_NONBLOCK );
		if ( f < 0 )
			continue;
		char name[128] = { 0 };
		if ( ioctl( f, EVIOCGNAME( sizeof( name ) - 1 ), name ) >= 0 &&
		     strstr( name, "PlayStation VR2 Headset" ) )
			fd = f;
		else
			close( f );
	}
	closedir( dir );
	return fd;
}

// Current dial value in metres, or <= 0 on failure.
static float ReadIpdMeters( int fd )
{
	if ( fd < 0 )
		return -1.0f;
	struct input_absinfo abs {};
	if ( ioctl( fd, EVIOCGABS( ABS_MISC ), &abs ) < 0 )
		return -1.0f;
	if ( abs.value < 50 || abs.value > 80 )
		return -1.0f;
	return abs.value * 1e-3f;
}

static const char *kSettingsSection = "driver_psvr2";

// PSVR2 panel EDID identity, decoded from the Sony SIE OSS panel EDID (see
// docs/references.md): manufacturer bytes 0x4D 0xD9 = "SNY", product id 0xA205,
// monitor name "SIE  VRH". SteamVR uses these to match and DRM-lease the
// physical connector in direct mode.
static constexpr int32_t kEdidVendorId = 0x4DD9;   // "SNY"
static constexpr int32_t kEdidProductId = 0xA205;

Psvr2HmdDriver::Psvr2HmdDriver()
{
	// Defaults can be overridden in resources/settings/default.vrsettings.
	// IVRSettings::GetString has no default-value arg, so read then fall back.
	char buf[256] = { 0 };
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	vr::VRSettings()->GetString( kSettingsSection, "model_number", buf, sizeof( buf ), &err );
	model_number_ = ( err == vr::VRSettingsError_None && buf[0] ) ? buf : "PSVR2";

	buf[0] = 0;
	err = vr::VRSettingsError_None;
	vr::VRSettings()->GetString( kSettingsSection, "serial_number", buf, sizeof( buf ), &err );
	serial_number_ = ( err == vr::VRSettingsError_None && buf[0] ) ? buf : "PSVR2-0001";

	const float hz = vr::VRSettings()->GetFloat( kSettingsSection, "display_frequency" );
	if ( hz > 0.0f )
		display_frequency_ = hz;

	Psvr2DisplayConfig cfg{};
	// Each field falls back to the struct default when the setting is absent (0).
	if ( int32_t v = vr::VRSettings()->GetInt32( kSettingsSection, "window_x" ) )      cfg.window_x = v;
	if ( int32_t v = vr::VRSettings()->GetInt32( kSettingsSection, "window_y" ) )      cfg.window_y = v;
	if ( int32_t v = vr::VRSettings()->GetInt32( kSettingsSection, "window_width" ) )  cfg.window_width = v;
	if ( int32_t v = vr::VRSettings()->GetInt32( kSettingsSection, "window_height" ) ) cfg.window_height = v;
	cfg.render_width = cfg.window_width / 2;
	cfg.render_height = cfg.window_height;

	// Direct mode is the default; an explicit "direct_mode": false in settings
	// switches to the extended-desktop scaffold.
	vr::EVRSettingsError dm_err = vr::VRSettingsError_None;
	const bool direct_mode = vr::VRSettings()->GetBool( kSettingsSection, "direct_mode", &dm_err );
	cfg.direct_mode = ( dm_err == vr::VRSettingsError_None ) ? direct_mode : true;
	direct_mode_ = cfg.direct_mode;

	// Per-unit factory optics from the headset; falls back to generic values.
	char calib_detail[256] = { 0 };
	Psvr2ReadFactoryCalibration( cfg.distortion_calibration, calib_detail, sizeof( calib_detail ) );
	DriverLog( "psvr2: %s", calib_detail );

	display_ = std::make_unique<Psvr2DisplayComponent>( cfg );
	pose_source_ = std::make_unique<PoseSource>();

	// Fusion tuning (all times in milliseconds; 0/absent keeps built-in defaults).
	const float slam_latency_ms = vr::VRSettings()->GetFloat( kSettingsSection, "slam_latency_ms" );
	const float fusion_tau_ms = vr::VRSettings()->GetFloat( kSettingsSection, "fusion_tau_ms" );
	if ( slam_latency_ms > 0.0f || fusion_tau_ms > 0.0f )
		pose_source_->SetFusionParams( slam_latency_ms > 0.0f ? slam_latency_ms * 1e-3 : -1.0,
		                               fusion_tau_ms > 0.0f ? fusion_tau_ms * 1e-3 : -1.0 );

	// Accel bias (wire axes, m/s^2, from tools/accel-bias-cal.py) + position tau.
	vr::EVRSettingsError berr = vr::VRSettingsError_None;
	const float b0 = vr::VRSettings()->GetFloat( kSettingsSection, "accel_bias_0", &berr );
	if ( berr == vr::VRSettingsError_None )
	{
		const float b1 = vr::VRSettings()->GetFloat( kSettingsSection, "accel_bias_1" );
		const float b2 = vr::VRSettings()->GetFloat( kSettingsSection, "accel_bias_2" );
		const float pos_tau_ms = vr::VRSettings()->GetFloat( kSettingsSection, "pos_tau_ms" );
		pose_source_->SetAccelParams( b0, b1, b2, pos_tau_ms > 0.0f ? pos_tau_ms * 1e-3 : -1.0 );
	}

	// EXPERIMENTAL: accel dead-reckoned translation; false = smoothed SLAM only.
	vr::EVRSettingsError aerr = vr::VRSettingsError_None;
	const bool accel_fusion = vr::VRSettings()->GetBool( kSettingsSection, "accel_fusion", &aerr );
	if ( aerr == vr::VRSettingsError_None )
		pose_source_->SetAccelFusion( accel_fusion );

	// Full accel matrix (row-major, 9 floats) from tools/accel-matrix-cal.py.
	double amat[9];
	bool have_mat = true;
	for ( int i = 0; i < 9 && have_mat; i++ )
	{
		char key[16];
		snprintf( key, sizeof( key ), "accel_mat_%d", i );
		vr::EVRSettingsError merr = vr::VRSettingsError_None;
		amat[i] = vr::VRSettings()->GetFloat( kSettingsSection, key, &merr );
		have_mat = ( merr == vr::VRSettingsError_None );
	}
	if ( have_mat )
	{
		pose_source_->SetAccelMatrix( amat );
		DriverLog( "psvr2: full accel matrix loaded from settings" );
	}

	if ( !pose_source_->HasPose() )
		DriverLog( "psvr2: /dev/psvr2-pose not found — HMD will not track (is the module loaded?)" );
}

vr::EVRInitError Psvr2HmdDriver::Activate( uint32_t unObjectId )
{
	device_index_ = unObjectId;

	vr::PropertyContainerHandle_t c = vr::VRProperties()->TrackedDeviceToPropertyContainer( unObjectId );
	vr::VRProperties()->SetStringProperty( c, vr::Prop_ModelNumber_String, model_number_.c_str() );
	vr::VRProperties()->SetStringProperty( c, vr::Prop_ManufacturerName_String, "Sony" );

	// IPD: prefer the headset's physical dial; fall back to the SteamVR setting.
	ipd_fd_ = OpenIpdInputDevice();
	const float dial_ipd = ReadIpdMeters( ipd_fd_ );
	const float ipd = ( dial_ipd > 0.0f )
	                      ? dial_ipd
	                      : vr::VRSettings()->GetFloat( vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_IPD_Float );
	if ( dial_ipd > 0.0f )
		DriverLog( "psvr2: IPD from headset dial: %.1f mm (live updates on)", dial_ipd * 1e3f );
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_UserIpdMeters_Float, ipd );
	last_ipd_ = ipd;

	// Required for the compositor to start.
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_DisplayFrequency_Float, display_frequency_ );
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.0f );
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_SecondsFromVsyncToPhotons_Float, 0.011f );

	// EDID identity lets SteamVR's compositor find the physical connector and
	// (in direct mode) DRM-lease it. In extended mode it's harmless.
	vr::VRProperties()->SetInt32Property( c, vr::Prop_EdidVendorID_Int32, kEdidVendorId );
	vr::VRProperties()->SetInt32Property( c, vr::Prop_EdidProductID_Int32, kEdidProductId );

	// IsOnDesktop must agree with the display component: false => SteamVR
	// acquires the panel directly (direct mode); true => extended desktop.
	vr::VRProperties()->SetBoolProperty( c, vr::Prop_IsOnDesktop_Bool, !direct_mode_ );

	DriverLog( "psvr2: display mode = %s (EDID %04X:%04X)",
	           direct_mode_ ? "direct (DRM lease)" : "extended desktop",
	           kEdidVendorId, kEdidProductId );

	active_ = true;
	pose_thread_ = std::thread( &Psvr2HmdDriver::PoseThread, this );
	return vr::VRInitError_None;
}

void Psvr2HmdDriver::Deactivate()
{
	if ( active_.exchange( false ) && pose_thread_.joinable() )
		pose_thread_.join();
	device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void Psvr2HmdDriver::EnterStandby() {}

void *Psvr2HmdDriver::GetComponent( const char *pchComponentNameAndVersion )
{
	if ( std::strcmp( pchComponentNameAndVersion, vr::IVRDisplayComponent_Version ) == 0 )
		return display_.get();
	return nullptr;
}

void Psvr2HmdDriver::DebugRequest( const char *, char *pchResponseBuffer, uint32_t unResponseBufferSize )
{
	if ( unResponseBufferSize >= 1 )
		pchResponseBuffer[0] = 0;
}

vr::DriverPose_t Psvr2HmdDriver::GetPose()
{
	return last_pose_;
}

void Psvr2HmdDriver::PoseThread()
{
	// With the IMU available, submit gyro-fused poses at ~500 Hz; otherwise
	// fall back to pacing on the blocking SLAM read (~stream rate).
	const bool fused = pose_source_->HasImu();
	if ( fused )
		DriverLog( "psvr2: IMU fusion active — gyro-integrated poses at ~500 Hz over the SLAM base" );

	unsigned tick = 0;
	while ( active_ )
	{
		// Live IPD-dial updates, ~1 Hz.
		if ( ipd_fd_ >= 0 && ( ++tick % 512 ) == 0 &&
		     device_index_ != vr::k_unTrackedDeviceIndexInvalid )
		{
			const float ipd = ReadIpdMeters( ipd_fd_ );
			if ( ipd > 0.0f && std::fabs( ipd - last_ipd_ ) >= 0.0005f )
			{
				vr::VRProperties()->SetFloatProperty(
				    vr::VRProperties()->TrackedDeviceToPropertyContainer( device_index_ ),
				    vr::Prop_UserIpdMeters_Float, ipd );
				DriverLog( "psvr2: IPD dial -> %.1f mm", ipd * 1e3f );
				last_ipd_ = ipd;
			}
		}

		vr::DriverPose_t pose{};
		const bool ok = fused ? pose_source_->GetFusedPose( pose )
		                      : pose_source_->ReadPose( pose );
		if ( ok )
		{
			last_pose_ = pose;
			if ( device_index_ != vr::k_unTrackedDeviceIndexInvalid )
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(
					device_index_, pose, sizeof( pose ) );
			if ( fused )
				std::this_thread::sleep_for( std::chrono::milliseconds( 2 ) );
		}
		else
		{
			// No node / no first sample yet / timeout: don't spin hot.
			std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
		}
	}
}

// ---------------------------------------------------------------------------
// Display component
// ---------------------------------------------------------------------------

Psvr2DisplayComponent::Psvr2DisplayComponent( const Psvr2DisplayConfig &config )
	: config_( config )
{
}

bool Psvr2DisplayComponent::IsDisplayOnDesktop()
{
	// Direct mode (default): NOT on the desktop, so SteamVR's compositor
	// acquires the panel directly via DRM leasing. Extended mode: on the desktop
	// as an ordinary monitor. See docs/steamvr.md.
	return !config_.direct_mode;
}

bool Psvr2DisplayComponent::IsDisplayRealDisplay()
{
	return true;
}

void Psvr2DisplayComponent::GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnWidth = config_.render_width;
	*pnHeight = config_.render_height;
}

void Psvr2DisplayComponent::GetEyeOutputViewport( vr::EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnY = 0;
	*pnWidth = config_.window_width / 2;
	*pnHeight = config_.window_height;
	*pnX = ( eEye == vr::Eye_Left ) ? 0 : config_.window_width / 2;
}

void Psvr2DisplayComponent::GetProjectionRaw( vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom )
{
	// Per-eye asymmetric frusta (Monado psvr2 branch: up/down 53deg,
	// outward 61.5deg, inward 43.5deg), as half-angle tangents.
	constexpr float kTanVert = 1.3270448f;    // tan(53deg)
	constexpr float kTanOutward = 1.8418131f; // tan(61.5deg)
	constexpr float kTanInward = 0.9489646f;  // tan(43.5deg)

	*pfTop = -kTanVert;
	*pfBottom = kTanVert;
	if ( eEye == vr::Eye_Left )
	{
		*pfLeft = -kTanOutward;
		*pfRight = kTanInward;
	}
	else
	{
		*pfLeft = -kTanInward;
		*pfRight = kTanOutward;
	}
}

vr::DistortionCoordinates_t Psvr2DisplayComponent::ComputeDistortion( vr::EVREye eEye, float fU, float fV )
{
	xrt_uv_triplet t{};
	psvr2_compute_distortion_asymmetric( config_.distortion_calibration, &t,
	                                     ( eEye == vr::Eye_Left ) ? 0 : 1, fU, fV );

	vr::DistortionCoordinates_t c{};
	c.rfRed[0] = t.r.x;
	c.rfRed[1] = t.r.y;
	c.rfGreen[0] = t.g.x;
	c.rfGreen[1] = t.g.y;
	c.rfBlue[0] = t.b.x;
	c.rfBlue[1] = t.b.y;
	return c;
}

void Psvr2DisplayComponent::GetWindowBounds( int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnX = config_.window_x;
	*pnY = config_.window_y;
	*pnWidth = config_.window_width;
	*pnHeight = config_.window_height;
}

bool Psvr2DisplayComponent::ComputeInverseDistortion( vr::HmdVector2_t *, vr::EVREye, uint32_t, float, float )
{
	// Let SteamVR infer the inverse from ComputeDistortion.
	return false;
}
