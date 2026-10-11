/*
 * SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: Copyright (c) 2024 Open Stick Community (gp2040-ce.info)
 */

#ifndef _USBDRIVER_CPP_
#define _USBDRIVER_CPP_

#include "tusb.h"
#include "drivermanager.h"
#include "uac1_speaker.h"
#include <string.h>

static bool usb_mounted;
static bool usb_suspended;

// ---------------------------------------------------------------------------
// UAC1 speaker (USB sound card) added next to the controller interfaces.
// - The UAC1 class driver is always registered (see usbd_app_driver_get_cb). Its open()
//   only claims an Audio-class function, so it does nothing in modes without audio.
// - The audio function is merged into the configuration descriptor only when the mode's
//   descriptor looks like XInput (interface class 0xFF / subclass 0x5D). This is decided
//   at enumeration time, because the driver's descriptor is not ready when TinyUSB starts.
// NOTE (Windows + XInput): XInput uses Microsoft's VID/PID 045E:028E, so Windows binds its own
// "Xbox 360 Controller for Windows" (XnaComposite) driver to the whole device and ignores extra
// interfaces. Test the sound card in a HID mode (DInput / PS4 / Switch) first.
// To enable another mode later, extend uac1_cfg_allowed().
// ---------------------------------------------------------------------------
// 1 = report the device as composite with IAD (class EF/02/01) when audio is merged in.
// Windows only splits a device into functions (controller + sound card) in that case.
#define UAC1_COMPOSITE_DEVICE_CLASS 1
// Master switch: 0 = behave exactly like stock GP2040-CE (use this to test the baseline)
#define UAC1_ENABLE 1
// PS5 EXPERIMENT (leave 0 normally): 1 = report the VID/PID of your standalone "Pico Sound Card"
// (2E8A:FEDD, which the PS5 accepted) instead of the mode's controller ID, while the audio function
// is merged in. The console will NOT accept the board as a controller in this state.
// It only answers: does the PS5 list the audio device when it does not recognise a controller?
#define UAC1_TEST_NON_SONY_ID 0
// PS5 EXPERIMENT 2 (leave 0 normally): 1 = make the whole device identical to the standalone
// "Pico Sound Card" (same device descriptor, config descriptor without HID and without IAD, same
// strings). The controller does NOT work in this state. If the PS5 lists the sound card now, the
// TinyUSB class driver is fine and the problem is the composite layout; if not, the class driver
// behaves differently from the pico-extras one.
#define UAC1_CLONE_STANDALONE 0

static bool uac1_drivers_ready = false;
static usbd_class_driver_t uac1_app_drivers[2];
static uint8_t uac1_cfg_buf[512] __attribute__((aligned(4)));
#if UAC1_COMPOSITE_DEVICE_CLASS || UAC1_TEST_NON_SONY_ID
static uint8_t uac1_dev_buf[18] __attribute__((aligned(4)));
#endif

#if UAC1_CLONE_STANDALONE
// Same values as blink.cpp: USB 1.1, class 0, EP0 64, 2E8A:FEDD, bcdDevice 0x0200, strings 1/2/3
static const uint8_t uac1_clone_dev[18] = {
	0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x40,
	0x8A, 0x2E, 0xDD, 0xFE, 0x00, 0x02, 0x01, 0x02, 0x03, 0x01
};
// Config: 2 interfaces, value 1, bmAttributes 0x80, 100 mA; audio function with itf 0/1, OUT 0x01, FB 0x82, no IAD
static const uint8_t uac1_clone_cfg[] = {
	0x09, 0x02, (9 + UAC1_SPK_FUNC_LEN) & 0xFF, (9 + UAC1_SPK_FUNC_LEN) >> 8, 0x02, 0x01, 0x00, 0x80, 0x32,
	UAC1_SPK_FUNC(0, 0x01, 0x82)
};
static uint16_t uac1_clone_str[33];
#endif

static bool uac1_cfg_allowed(const uint8_t *cfg) {
	if (!cfg) return false;
	const uint16_t len = cfg[2] | (cfg[3] << 8);
	if (len < 9) return false;   // descriptor not filled in (yet)
	for (uint16_t i = cfg[0]; i + 9 <= len && cfg[i]; i += cfg[i]) {
		if (cfg[i + 1] != 0x04) continue;                      // only look at interface descriptors
		if (cfg[i + 5] == 0xFF && cfg[i + 6] == 0x5D) return true;  // XInput (Windows' Xbox driver owns this device, see notes)
		if (cfg[i + 5] == 0x03) return true;                  // HID: DInput, PS3, PS4, Switch, keyboard ...
	}
	return false;
}

bool get_usb_mounted(void) {
	return usb_mounted;
}

bool get_usb_suspended(void) {
	return usb_suspended;
}

const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *driver_count) {
	const usbd_class_driver_t *mode = DriverManager::getInstance().getDriver()->get_class_driver();
#if UAC1_ENABLE
	if (mode) {
		if (!uac1_drivers_ready) {
			uac1_app_drivers[0] = *mode;
			uac1_app_drivers[1] = *uac1_speaker_class_driver();
			uac1_drivers_ready = true;
		}
		*driver_count = 2;
		return uac1_app_drivers;
	}
#endif
	*driver_count = 1;
	return mode;
}

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
	return DriverManager::getInstance().getDriver()->get_report(report_id, report_type, buffer, reqlen);
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
	DriverManager::getInstance().getDriver()->set_report(report_id, report_type, buffer, bufsize);
}

// Invoked when device is mounted
void tud_mount_cb(void)
{
	usb_mounted = true;
	usb_suspended = false;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	usb_mounted = false;
	usb_suspended = false;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
	(void)remote_wakeup_en;
	usb_suspended = true;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
	usb_suspended = false;
}

// Vendor Controlled XFER occured
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request) {
	return DriverManager::getInstance().getDriver()->vendor_control_xfer_cb(rhport, stage, request);
}


// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
#if UAC1_ENABLE && UAC1_CLONE_STANDALONE
	(void)langid;
	const char *str = nullptr;
	switch (index) {
		case 0: uac1_clone_str[0] = (TUSB_DESC_STRING << 8) | 4; uac1_clone_str[1] = 0x0409; return uac1_clone_str;
		case 1: str = "Raspberry Pi"; break;
		case 2: str = "Pico Sound Card"; break;
		case 3: str = "0123456789AB"; break;
		default: return nullptr;
	}
	uint8_t n = (uint8_t)strlen(str);
	for (uint8_t i = 0; i < n; i++) uac1_clone_str[1 + i] = str[i];
	uac1_clone_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
	return uac1_clone_str;
#endif
	return DriverManager::getInstance().getDriver()->get_descriptor_string_cb(index, langid);
}

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb() {
#if UAC1_ENABLE && UAC1_CLONE_STANDALONE
	return uac1_clone_dev;
#endif
	const uint8_t *dev = DriverManager::getInstance().getDriver()->get_descriptor_device_cb();
#if UAC1_ENABLE && (UAC1_COMPOSITE_DEVICE_CLASS || UAC1_TEST_NON_SONY_ID)
	if (dev) {
		const bool merged = uac1_cfg_allowed(DriverManager::getInstance().getDriver()->get_descriptor_configuration_cb(0));
		bool patched = false;
		(void)merged;
		memcpy(uac1_dev_buf, dev, sizeof(uac1_dev_buf));
#if UAC1_COMPOSITE_DEVICE_CLASS
		// Only vendor-class device descriptors (0xFF, e.g. XInput) need the composite/IAD class;
		// HID modes already use class 0, which Windows treats as composite.
		if (merged && dev[4] == 0xFF) {
			uac1_dev_buf[4] = 0xEF;  // bDeviceClass: miscellaneous
			uac1_dev_buf[5] = 0x02;  // bDeviceSubClass: common
			uac1_dev_buf[6] = 0x01;  // bDeviceProtocol: IAD
			patched = true;
		}
#endif
#if UAC1_TEST_NON_SONY_ID
		// Experiment: applied in every mode, independent of the descriptor check above.
		uac1_dev_buf[8]  = 0x8A;  // idVendor  0x2E8A (Raspberry Pi)
		uac1_dev_buf[9]  = 0x2E;
		uac1_dev_buf[10] = 0xDD;  // idProduct 0xFEDD (same as the working Pico Sound Card)
		uac1_dev_buf[11] = 0xFE;
		patched = true;
#endif
		if (patched) return uac1_dev_buf;
	}
#endif
	return dev;
}

// Invoked when received GET HID REPORT DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t itf) {
	return DriverManager::getInstance().getDriver()->get_hid_descriptor_report_cb(itf);
}

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
#if UAC1_ENABLE && UAC1_CLONE_STANDALONE
	(void)index;
	return uac1_clone_cfg;
#endif
	const uint8_t *cfg = DriverManager::getInstance().getDriver()->get_descriptor_configuration_cb(index);
#if UAC1_ENABLE
	if (uac1_cfg_allowed(cfg) && uac1_speaker_build_config(cfg, uac1_cfg_buf, sizeof(uac1_cfg_buf)) != 0) {
		return uac1_cfg_buf;
	}
#endif
	return cfg;
}

uint8_t const* tud_descriptor_device_qualifier_cb() {
#if UAC1_ENABLE && UAC1_CLONE_STANDALONE
	return nullptr;
#endif
	return DriverManager::getInstance().getDriver()->get_descriptor_device_qualifier_cb();
}

#endif
