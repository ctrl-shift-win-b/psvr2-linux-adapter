// SPDX-License-Identifier: GPL-2.0
//
// Factory calibration readout. The headset stores per-unit optical calibration
// retrievable via a vendor control transfer on ep0 (report 0x8f) — protocol and
// parameter remap ported from Monado's psvr2-work-steamvr branch (BSL-1.0,
// psvr2.c: get_psvr2_control / psvr2_setup_distortion_and_fovs).
//
// ep0 control transfers don't require claiming an interface, so this coexists
// with the psvr2 kernel module owning the vendor interfaces. It does require
// permission to open the usbfs device node (udev uaccess rule).

#include "calibration.h"

#include <libusb-1.0/libusb.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{

constexpr uint16_t kVendorSony = 0x054c;
constexpr uint16_t kProductPsvr2 = 0x0cde;
constexpr uint16_t kReportCalibration = 0x8f;

#pragma pack( push, 1 )
struct SieCtrlPkt
{
	uint16_t report_id;
	uint16_t subcmd;
	uint32_t len;
	uint8_t data[0x100];
};

struct CalibrationBlock
{
	uint8_t version_unk;
	uint8_t unk[7];
	float distortion_params[32];
};
#pragma pack( pop )

// Nominal values when no per-unit data is available: Monado's version<4
// offsets, but identity tilt rotations (cos=1, sin=0). Monado's own fallback
// leaves the cos/sin slots zeroed, which degenerates every distorted
// coordinate to the eye centre — we deviate deliberately.
constexpr float kFallback[8] = { -0.09919293f, 0.0f, 0.09919293f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f };

} // namespace

bool Psvr2ReadFactoryCalibration( float out[8], char *detail, size_t detail_len )
{
	std::memcpy( out, kFallback, sizeof( kFallback ) );

	libusb_context *ctx = nullptr;
	if ( libusb_init( &ctx ) != 0 )
	{
		std::snprintf( detail, detail_len, "libusb init failed — using fallback calibration" );
		return false;
	}

	libusb_device_handle *dev = libusb_open_device_with_vid_pid( ctx, kVendorSony, kProductPsvr2 );
	if ( !dev )
	{
		std::snprintf( detail, detail_len,
		               "cannot open USB device %04x:%04x (udev uaccess rule missing?) — using fallback calibration",
		               kVendorSony, kProductPsvr2 );
		libusb_exit( ctx );
		return false;
	}

	SieCtrlPkt pkt{};
	pkt.report_id = kReportCalibration;
	pkt.subcmd = 1;
	pkt.len = sizeof( pkt.data );

	const int ret = libusb_control_transfer(
	    dev, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT | LIBUSB_ENDPOINT_IN, 0x1, kReportCalibration,
	    0x0, reinterpret_cast<unsigned char *>( &pkt ), sizeof( pkt ), 100 );

	libusb_close( dev );
	libusb_exit( ctx );

	if ( ret < 0 )
	{
		std::snprintf( detail, detail_len, "control transfer failed (%d) — using fallback calibration", ret );
		return false;
	}

	CalibrationBlock block{};
	std::memcpy( &block, pkt.data, sizeof( block ) );

	if ( block.version_unk < 4 )
	{
		std::snprintf( detail, detail_len, "calibration block version %u < 4 — using fallback calibration",
		               block.version_unk );
		return false;
	}

	// Remap ported verbatim from Monado psvr2_setup_distortion_and_fovs.
	const float *p = block.distortion_params;

	out[0] = ( ( ( -p[0] - p[6] ) * 29.9f + 14.95f ) / 1000.0f - 3.22f ) / 32.46199f;
	out[1] = ( ( ( -p[1] * 29.9f ) + 14.95f ) / 1000.0f ) / 32.46199f;

	out[2] = ( ( ( p[6] - p[2] ) * 29.9f + 14.95f ) / 1000.0f + 3.22f ) / 32.46199f;
	out[3] = ( ( ( -p[3] * 29.9f ) + 14.95f ) / 1000.0f ) / 32.46199f;

	const float left = -p[4] * static_cast<float>( M_PI ) / 180.0f;
	out[4] = std::cos( left );
	out[5] = std::sin( left );

	const float right = -p[5] * static_cast<float>( M_PI ) / 180.0f;
	out[6] = std::cos( right );
	out[7] = std::sin( right );

	std::snprintf( detail, detail_len,
	               "factory calibration v%u loaded: off L(%.5f, %.5f) R(%.5f, %.5f) tilt L(%.4f) R(%.4f) deg",
	               block.version_unk, out[0], out[1], out[2], out[3], -p[4], -p[5] );
	return true;
}
