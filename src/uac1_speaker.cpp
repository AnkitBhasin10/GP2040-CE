// UAC1 speaker class driver for TinyUSB.
// Audio logic is copied unchanged from the working pico-extras blink.cpp:
//   - same buffer pool (32 x 512), same DC-offset filter + clipping, same feedback formula
//   - same I2S setup (connect_extra false, 2, 96)
// Only changed for coexistence with GP2040-CE:
//   1) USB layer: pico-extras usb_device -> TinyUSB custom class driver
//   2) no main(), no set_sys_clock_khz(), no multicore_launch_core1() (GP2040-CE owns those)
//   3) DMA channel / PIO state machine are picked free instead of hardcoded 0 / 0
// NOT compile-tested: check the [verify] notes against your TinyUSB version.

#include "uac1_speaker.h"

#include <string.h>

#include "pico/stdlib.h"
#include "pico/audio.h"
#include "pico/audio_i2s.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/sync.h"

// 1 = set up I2S (PIO, DMA, DMA interrupt) on core 1 instead of in the USB task on core 0.
// Core 1 picks the request up in uac1_speaker_core1_poll(), called from GP2040Aux::run().
// The USB packet handling itself must stay on core 0 (TinyUSB is single-core).
#ifndef UAC1_I2S_ON_CORE1
#define UAC1_I2S_ON_CORE1 1
#endif

#define UAC1_START_I2S 0
#define current_sample_rate 48000
#define USB_FEEDBACK 1
#define BUFFER_NUM 16                 // as in the original (pool below is 32, kept as-is)
#define UAC1_OUT_MAX_PKT 196          // AUDIO_MAX_PACKET_SIZE(48000) with feedback

#define FEATURE_MUTE_CONTROL    1u
#define FEATURE_VOLUME_CONTROL  2u
#define ENDPOINT_FREQ_CONTROL   1u
#define UAC1_UNIT_FEATURE       2

#define AUDIO_REQ_SET_CUR  0x01
#define AUDIO_REQ_GET_CUR  0x81
#define AUDIO_REQ_GET_MIN  0x82
#define AUDIO_REQ_GET_MAX  0x83
#define AUDIO_REQ_GET_RES  0x84

#define ENCODE_DB(x)    ((int16_t)((x) * 256))
#define MIN_VOLUME      ENCODE_DB(-91)
#define MAX_VOLUME      ENCODE_DB(0)
#define VOLUME_RESOLUTION ENCODE_DB(1)

static audio_buffer_pool_t * volatile producer_pool;   // set last, by whichever core runs init
static volatile bool i2s_init_requested = false;
static volatile bool i2s_init_done = false;

static uint8_t itf_as, alt;
static uint8_t ep_out, ep_fb;
static tusb_desc_endpoint_t ep_out_desc, ep_fb_desc;

static uint8_t out_buf[UAC1_OUT_MAX_PKT] __attribute__((aligned(4)));
static uint8_t fb_buf[4] __attribute__((aligned(4)));
static uint8_t ctrl_buf[4] __attribute__((aligned(4)));

// Same state as the original. Note: like the original, volume/mute are tracked and
// reported to the host but not applied to the samples (refactor later).
static struct {
    uint32_t freq;
    int16_t volume;
    bool mute;
} audio_state = { current_sample_rate, 0, false };

#if USB_FEEDBACK
// for USB-audio rate feedback, number of vacant buffers is used (unchanged)
static int countFreeBuffers() {
    if (!producer_pool) return BUFFER_NUM / 2;   // I2S not started: report "balanced"
    int i = 0;
    audio_buffer_t *audio_buffer = producer_pool->free_list;
    while (audio_buffer != nullptr) {
        audio_buffer = audio_buffer->next;
        i++;
    }
    return i;
}
#endif

// ---- was _as_sync_packet ----
static void send_feedback(uint8_t rhport) {
    if (usbd_edpt_busy(rhport, ep_fb)) return;
#if USB_FEEDBACK
    // calc rate adjustment value between -40 to 40 (unchanged)
    int feedbackvalue = (countFreeBuffers() - BUFFER_NUM / 2) * (2 * 40 / BUFFER_NUM);
    uint feedback = ((audio_state.freq + feedbackvalue) << 14u) / 1000u;
#else
    uint feedback = (audio_state.freq << 14u) / 1000u;
#endif
    fb_buf[0] = feedback;
    fb_buf[1] = feedback >> 8u;
    fb_buf[2] = feedback >> 16u;
    usbd_edpt_xfer(rhport, ep_fb, fb_buf, 3);
}

// ---- was _as_audio_packet ----
static uint32_t underrun_count = 0;

static void handle_out(uint8_t rhport, uint32_t n) {
    if (!producer_pool) { if (alt) usbd_edpt_xfer(rhport, ep_out, out_buf, UAC1_OUT_MAX_PKT); return; }
    struct audio_buffer *audio_buffer = take_audio_buffer(producer_pool, false);
    if (!audio_buffer) {
        underrun_count++;   // original printf'd here; removed (printf in IRQ context)
    } else {
        const uint32_t sample_count = n / 4;
        audio_buffer->sample_count = sample_count;

        int16_t *out = (int16_t *)audio_buffer->buffer->bytes;
        const int16_t *in = (const int16_t *)out_buf;

        static int32_t dc_offset = 0;
        for (uint32_t i = 0; i < sample_count * 2; i++) {
            int32_t sample = in[i];
            dc_offset = (dc_offset * 31 + sample) / 32;
            out[i] = (int16_t)(sample - dc_offset);

            if (out[i] > 32760) out[i] = 32760;
            if (out[i] < -32760) out[i] = -32760;
        }
        give_audio_buffer(producer_pool, audio_buffer);
    }
    if (alt) usbd_edpt_xfer(rhport, ep_out, out_buf, UAC1_OUT_MAX_PKT);
}

// ---- was as_set_alternate ----
static bool set_alt(uint8_t rhport, uint8_t a) {
    if (a >= 2) return false;
    if (a == alt) return true;
    if (alt) {
        usbd_edpt_close(rhport, ep_out);   // [verify] needs a TinyUSB with usbd_edpt_close
        usbd_edpt_close(rhport, ep_fb);
    }
    alt = a;
    if (alt) {
        usbd_edpt_open(rhport, &ep_out_desc);   // [verify] some versions: usbd_edpt_iso_activate()
        usbd_edpt_open(rhport, &ep_fb_desc);
        usbd_edpt_xfer(rhport, ep_out, out_buf, UAC1_OUT_MAX_PKT);
        send_feedback(rhport);
    }
    return true;
}

// ---- Class driver callbacks ----
static void drv_init(void) { alt = 0; }
static void drv_reset(uint8_t rhport) { (void)rhport; alt = 0; }

// copy the OUT and feedback endpoint descriptors found in [p, end)
static void claim_endpoints(uint8_t const *p, uint8_t const *end) {
    while (p < end) {
        if (tu_desc_type(p) == TUSB_DESC_ENDPOINT) {
            tusb_desc_endpoint_t const *e = (tusb_desc_endpoint_t const *)p;
            if (tu_edpt_dir(e->bEndpointAddress) == TUSB_DIR_OUT) {
                memcpy(&ep_out_desc, e, sizeof(ep_out_desc));
                ep_out = e->bEndpointAddress;
            } else {
                memcpy(&ep_fb_desc, e, sizeof(ep_fb_desc));
                ep_fb = e->bEndpointAddress;
            }
        }
        p = tu_desc_next(p);
    }
}

// First enumeration of the audio function: bring up I2S (runs in the USB task, not an IRQ)
#ifndef UAC1_START_I2S
#define UAC1_START_I2S 1   // 0 = enumerate the sound card but don't touch PIO/DMA/RAM (debug aid)
#endif
static void request_i2s_start(void) {
#if UAC1_START_I2S
    static bool requested = false;
    if (!requested) {
        requested = true;
#if UAC1_I2S_ON_CORE1
        i2s_init_requested = true;   // core 1 runs uac1_speaker_init(); packets are dropped until it is ready
#else
        uac1_speaker_init();
#endif
    }
#endif
}

// true if an Interface Association Descriptor for this interface sits right in front of it
static bool preceded_by_iad(tusb_desc_interface_t const *d) {
    uint8_t const *p = (uint8_t const *)d;
    return p[-8] == 0x08 && p[-7] == TUSB_DESC_INTERFACE_ASSOCIATION && p[-6] == d->bInterfaceNumber;
}

static uint16_t drv_open(uint8_t rhport, tusb_desc_interface_t const *desc_itf, uint16_t max_len) {
    (void)rhport;
    if (desc_itf->bInterfaceClass != 0x01) return 0;
    uint8_t const *base = (uint8_t const *)desc_itf;

    if (desc_itf->bInterfaceSubClass == 0x01) {            // Audio Control
        if (preceded_by_iad(desc_itf)) {
            // with an IAD one open() covers the whole function (control + streaming)
            if (max_len < UAC1_SPK_FUNC_LEN) return 0;
            itf_as = desc_itf->bInterfaceNumber + 1;
            claim_endpoints(base, base + UAC1_SPK_FUNC_LEN);
            request_i2s_start();
            return UAC1_SPK_FUNC_LEN;
        }
        // no IAD (like the standalone device): TinyUSB calls open() again for the streaming interface
        if (max_len < UAC1_AC_LEN) return 0;
        return UAC1_AC_LEN;
    }

    if (desc_itf->bInterfaceSubClass == 0x02) {            // Audio Streaming (only reached without IAD)
        if (max_len < UAC1_AS_LEN) return 0;
        itf_as = desc_itf->bInterfaceNumber;
        claim_endpoints(base, base + UAC1_AS_LEN);
        request_i2s_start();
        return UAC1_AS_LEN;
    }
    return 0;
}

// ---- was ac/as setup handlers + do_get_*/do_set_current/audio_cmd_packet ----
static bool drv_control(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
        if (req->bRequest == TUSB_REQ_SET_INTERFACE) {
            if (stage == CONTROL_STAGE_SETUP) {
                if (tu_u16_low(req->wIndex) != itf_as) return false;
                if (!set_alt(rhport, (uint8_t)tu_u16_low(req->wValue))) return false;
                return tud_control_status(rhport, req);
            }
            return true;
        }
        if (req->bRequest == TUSB_REQ_GET_INTERFACE) {
            if (stage != CONTROL_STAGE_SETUP) return true;
            ctrl_buf[0] = alt;
            return tud_control_xfer(rhport, req, ctrl_buf, 1);
        }
        return false;
    }

    if (req->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS) return false;

    uint8_t cs = tu_u16_high(req->wValue);
    uint16_t len;
    bool to_endpoint = (req->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT);

    if (!to_endpoint) {
        if (tu_u16_high(req->wIndex) != UAC1_UNIT_FEATURE) return false;
        if (cs == FEATURE_MUTE_CONTROL) len = 1;
        else if (cs == FEATURE_VOLUME_CONTROL) len = 2;
        else return false;
    } else {
        if (cs != ENDPOINT_FREQ_CONTROL) return false;
        len = 3;
    }

    if (req->bRequest == AUDIO_REQ_SET_CUR) {
        if (stage == CONTROL_STAGE_SETUP) {
            if (req->wLength != len) return false;
            return tud_control_xfer(rhport, req, ctrl_buf, len);
        }
        if (stage == CONTROL_STAGE_DATA) {
            if (to_endpoint) {
                // original: any frequency request ends up forced to 48000
                audio_state.freq = 48000;
            } else if (cs == FEATURE_MUTE_CONTROL) {
                audio_state.mute = ctrl_buf[0];
            } else {
                audio_state.volume = (int16_t)(ctrl_buf[0] | (ctrl_buf[1] << 8));
            }
        }
        return true;
    }

    if (stage != CONTROL_STAGE_SETUP) return true;

    int32_t v;
    switch (req->bRequest) {
        case AUDIO_REQ_GET_CUR:
            if (to_endpoint) v = audio_state.freq;
            else v = (cs == FEATURE_MUTE_CONTROL) ? audio_state.mute : audio_state.volume;
            break;
        case AUDIO_REQ_GET_MIN:
            if (to_endpoint || cs != FEATURE_VOLUME_CONTROL) return false;
            v = MIN_VOLUME;
            break;
        case AUDIO_REQ_GET_MAX:
            if (to_endpoint || cs != FEATURE_VOLUME_CONTROL) return false;
            v = MAX_VOLUME;
            break;
        case AUDIO_REQ_GET_RES:
            if (to_endpoint || cs != FEATURE_VOLUME_CONTROL) return false;
            v = VOLUME_RESOLUTION;
            break;
        default:
            return false;
    }
    ctrl_buf[0] = (uint8_t)v;
    ctrl_buf[1] = (uint8_t)(v >> 8);
    ctrl_buf[2] = (uint8_t)(v >> 16);
    return tud_control_xfer(rhport, req, ctrl_buf, len);
}

static bool drv_xfer(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred) {
    (void)result;
    if (ep_addr == ep_out) { handle_out(rhport, xferred); return true; }
    if (ep_addr == ep_fb)  { if (alt) send_feedback(rhport); return true; }
    return false;
}

static usbd_class_driver_t const uac1_driver = {
#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
    .name = "UAC1-SPK",
#endif
    .init            = drv_init,
    .reset           = drv_reset,
    .open            = drv_open,
    .control_xfer_cb = drv_control,
    .xfer_cb         = drv_xfer,
    .sof             = NULL,
};

extern "C" usbd_class_driver_t const *uac1_speaker_class_driver(void) {
    return &uac1_driver;
}

// ---- was init_audio(), minus usb_sound_card_init() and core1 ----
extern "C" void uac1_speaker_init(void) {
    audio_i2s_set_enabled(false);

    static audio_format_t audio_format = {
        .sample_freq = current_sample_rate,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2,
    };

    static struct audio_buffer_format producer_format = {
        .format = &audio_format,
        .sample_stride = 4
    };

    audio_buffer_pool_t *pool = audio_new_producer_pool(&producer_format, 32, 512);  // unchanged (64 KB)

    // pico_audio_i2s claims these itself: pick free ones instead of hardcoded 0 / 0.
    // If GP2040-CE's LED code uses PIO0, set PICO_AUDIO_I2S_PIO=1 in the build.
    uint dma_ch = dma_claim_unused_channel(true);
    dma_channel_unclaim(dma_ch);
    PIO pio = (PICO_AUDIO_I2S_PIO == 0) ? pio0 : pio1;
    uint sm = pio_claim_unused_sm(pio, true);
    pio_sm_unclaim(pio, sm);

    struct audio_i2s_config config = {
        .data_pin = UAC1_I2S_DATA_PIN,
        .clock_pin_base = UAC1_I2S_CLOCK_PIN_BASE,
        .dma_channel = (uint8_t)dma_ch,
        .pio_sm = (uint8_t)sm,
    };

    const struct audio_format *output_format = audio_i2s_setup(&audio_format, &config);
    if (!output_format) {
        panic("audio_i2s_setup failed");
    }

    bool __unused ok = audio_i2s_connect_extra(pool, false, 2, 96, nullptr);
    assert(ok);

    audio_i2s_set_enabled(true);

    // publish the pool only now: the USB code on core 0 ignores audio packets while it is NULL
    __dmb();
    producer_pool = pool;
}

// Called repeatedly from core 1 (GP2040Aux::run). Cheap when nothing is pending.
extern "C" void uac1_speaker_core1_poll(void) {
#if UAC1_I2S_ON_CORE1
    if (i2s_init_requested && !i2s_init_done) {
        i2s_init_done = true;
        uac1_speaker_init();
    }
#endif
}

// ---- Configuration descriptor merge helper ----
extern "C" uint16_t uac1_speaker_build_config(const uint8_t *base, uint8_t *out, uint16_t out_max) {
    const uint16_t base_len = base[2] | (base[3] << 8);
    if (base_len < 9) return 0;   // base descriptor not filled in
    if ((uint32_t)base_len + UAC1_SPK_DESC_LEN > out_max) return 0;

    memcpy(out, base, base_len);

    // endpoint numbers already used by the base descriptor (either direction)
    uint16_t used = 0;
    for (const uint8_t *p = base + base[0]; p < base + base_len && p[0]; p += p[0]) {
        if (p[1] == TUSB_DESC_ENDPOINT) used |= (uint16_t)(1u << (p[2] & 0x0F));
    }
    uint8_t epo = 0, epf = 0;
    for (uint8_t n = 1; n < 16; n++) {
        if (used & (1u << n)) continue;
        if (!epo) epo = n;
        else { epf = n; break; }
    }
    if (!epo || !epf) return 0;

    const uint8_t first_itf = base[4];          // next free interface number
    const uint8_t ep_out_addr = epo;            // OUT
    const uint8_t ep_fb_addr = (uint8_t)(epf | 0x80);  // IN

    const uint8_t audio[UAC1_SPK_DESC_LEN] = {
        UAC1_SPK_DESCRIPTOR(first_itf, ep_out_addr, ep_fb_addr)
    };
    memcpy(out + base_len, audio, sizeof(audio));

    const uint16_t total = base_len + UAC1_SPK_DESC_LEN;
    out[2] = (uint8_t)total;
    out[3] = (uint8_t)(total >> 8);
    out[4] = base[4] + UAC1_SPK_NUM_ITF;
    return total;
}
