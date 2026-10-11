// UAC1 stereo 16-bit 48 kHz USB speaker for TinyUSB (custom class driver)
// Output: PCM5102 (or any I2S DAC) through pico_audio_i2s (pico-extras)
#pragma once

#include <stdint.h>
#include "tusb.h"
#include "device/usbd_pvt.h"

// ---- Pins (adjust to your board config) ----
#define UAC1_I2S_DATA_PIN        22   // DIN
#define UAC1_I2S_CLOCK_PIN_BASE  26   // BCK = 26, LRCK = 27 (must be consecutive)

// ---- Descriptor ----
// The audio function is byte-identical to the working standalone blink.cpp device.
//   _itf   : first free interface number (audio uses _itf and _itf+1)
//   _epout : iso OUT endpoint address, e.g. 0x01
//   _epfb  : iso feedback IN endpoint address, e.g. 0x82
// UAC1_USE_IAD = 1 puts an Interface Association Descriptor in front (needed by some hosts for
// composite devices); 0 = no IAD, exactly like the standalone device. The class driver handles both.
// Add UAC1_SPK_NUM_ITF to bNumInterfaces and UAC1_SPK_DESC_LEN to wTotalLength.
#ifndef UAC1_USE_IAD
#define UAC1_USE_IAD 0   // 0 = like the working standalone device; 1 = add an IAD
#endif

#define UAC1_SPK_NUM_ITF    2
#define UAC1_SPK_FUNC_LEN   113   // control + streaming interfaces, without IAD
#define UAC1_AC_LEN         49    // audio control interface + its class-specific descriptors
#define UAC1_AS_LEN         64    // audio streaming interface (alt 0 + alt 1) + endpoints

#define UAC1_SPK_IAD(_itf) \
  0x08, 0x0B, (_itf), 0x02, 0x01, 0x00, 0x00, 0x00

#define UAC1_SPK_FUNC(_itf, _epout, _epfb) \
  /* AC interface */ \
  0x09, 0x04, (_itf), 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, \
  /* AC header: bcdADC 1.00, wTotalLength 40, 1 streaming interface */ \
  0x09, 0x24, 0x01, 0x00, 0x01, 0x28, 0x00, 0x01, (uint8_t)((_itf) + 1), \
  /* Input terminal: USB streaming, 2 ch, L+R */ \
  0x0C, 0x24, 0x02, 0x01, 0x01, 0x01, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, \
  /* Feature unit: master mute + volume */ \
  0x0A, 0x24, 0x06, 0x02, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, \
  /* Output terminal: speaker */ \
  0x09, 0x24, 0x03, 0x03, 0x01, 0x03, 0x00, 0x02, 0x00, \
  /* AS interface alt 0 */ \
  0x09, 0x04, (uint8_t)((_itf) + 1), 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, \
  /* AS interface alt 1 (OUT + feedback) */ \
  0x09, 0x04, (uint8_t)((_itf) + 1), 0x01, 0x02, 0x01, 0x02, 0x00, 0x00, \
  /* AS general: terminal link 1, delay 1, PCM */ \
  0x07, 0x24, 0x01, 0x01, 0x01, 0x01, 0x00, \
  /* Format type I: 2 ch, 2 bytes, 16 bit, 2 freq entries (48000, 0) like the original */ \
  0x0E, 0x24, 0x02, 0x01, 0x02, 0x02, 0x10, 0x02, 0x80, 0xBB, 0x00, 0x00, 0x00, 0x00, \
  /* Iso OUT endpoint (async), 196 bytes, sync address = feedback EP */ \
  0x09, 0x05, (_epout), 0x05, 0xC4, 0x00, 0x01, 0x00, (_epfb), \
  /* Class-specific audio endpoint: sampling-frequency control */ \
  0x07, 0x25, 0x01, 0x01, 0x00, 0x00, 0x00, \
  /* Iso feedback IN endpoint */ \
  0x09, 0x05, (_epfb), 0x11, 0x03, 0x00, 0x01, 0x02, 0x00

#if UAC1_USE_IAD
#define UAC1_SPK_DESC_LEN  (UAC1_SPK_FUNC_LEN + 8)
#define UAC1_SPK_DESCRIPTOR(_itf, _epout, _epfb) UAC1_SPK_IAD(_itf), UAC1_SPK_FUNC(_itf, _epout, _epfb)
#else
#define UAC1_SPK_DESC_LEN  UAC1_SPK_FUNC_LEN
#define UAC1_SPK_DESCRIPTOR(_itf, _epout, _epfb) UAC1_SPK_FUNC(_itf, _epout, _epfb)
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Call once at startup (sets up buffer pool, I2S PIO + DMA)
void uac1_speaker_init(void);

// Builds "base configuration descriptor + audio function" into out[].
//   base   : the mode's existing full configuration descriptor (starts with 09 02 ...)
//   out    : static buffer that outlives the USB transfer (>= base length + UAC1_SPK_DESC_LEN)
// It appends the audio function, picks two unused endpoint numbers, uses the next free
// interface numbers, and patches wTotalLength / bNumInterfaces. Returns the new total
// length, or 0 if out_max is too small. Assumes the base interfaces are numbered 0..n-1.
uint16_t uac1_speaker_build_config(const uint8_t* base, uint8_t* out, uint16_t out_max);

// Call from the core 1 loop (GP2040Aux::run). Starts the I2S output on core 1 once the
// host has enumerated the audio function. Does nothing if UAC1_I2S_ON_CORE1 is 0.
void uac1_speaker_core1_poll(void);

// Return this from usbd_app_driver_get_cb() (append to existing drivers)
usbd_class_driver_t const* uac1_speaker_class_driver(void);

#ifdef __cplusplus
}
#endif
