// SPDX-License-Identifier: CC0-1.0

#include "tusb_option.h"

#include "driver.h"
#include "device/usbd_pvt.h"

#include "gud.h"
#include "lz4.h"

#include "esp_timer.h"

#include "esp_log.h"
#include <inttypes.h>
#include <stdio.h>
static const char* TAG = "Driver.c";

#define GUD_DRV_LOG1
#define GUD_DRV_LOG2

#define GUD_DRV_FLUSH_STATS     0

#define GUD_CTRL_REQ_BUF_SIZE   128 // Fits EDID

#define CFG_GUD_BULK_OUT_SIZE 64
// Max usbd_edpt_xfer() xfer size is uint16_t, align to endpoint size
#define GUD_EDPT_XFER_MAX_SIZE  (0xffff - (0xffff % CFG_GUD_BULK_OUT_SIZE))

#define min(a,b)    (((a) < (b)) ? (a) : (b))

typedef struct
{
    uint8_t itf_num;
    uint8_t ep_out;

    uint8_t *buf;
    uint32_t xfer_len;
    uint32_t len;
    uint32_t offset;
} gud_interface_t;

CFG_TUSB_MEM_SECTION static gud_interface_t _gud_itf;

const struct gud_display *_display;
uint8_t *_framebuffer;
uint8_t *_compress_buf;

CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN static uint8_t _ctrl_req_buf[GUD_CTRL_REQ_BUF_SIZE];
CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN static uint8_t status;

static bool controller_enabled;
static uint64_t req_timestamp;

#if GUD_DRV_FLUSH_STATS
typedef struct
{
    uint64_t ctrl_start;
    uint64_t bulk_start;
    uint64_t bulk_end;
    uint64_t decompress_end;
    uint64_t write_end;
} gud_driver_flush_stat_t;

gud_driver_flush_stat_t _flush_stat;

#define gud_driver_flush_stat_set(m)    _flush_stat.m = time_us_64();

static void gud_driver_flush_stat_start()
{
    memset(&_flush_stat, 0, sizeof(_flush_stat));
    gud_driver_flush_stat_set(ctrl_start);
}

static void gud_driver_flush_stat_print()
{
    uint64_t write_begin = _flush_stat.bulk_end;

    printf("C:%llu + B:%llu",
           _flush_stat.bulk_start - _flush_stat.ctrl_start,
           _flush_stat.bulk_end - _flush_stat.bulk_start);

    if (_flush_stat.decompress_end) {
        printf(" + D:%llu", _flush_stat.decompress_end - _flush_stat.bulk_end);
        write_begin = _flush_stat.decompress_end;
    }

    printf(" = %llu + W:%llu = %llu us\n",
           write_begin - _flush_stat.ctrl_start,
           _flush_stat.write_end - write_begin,
           _flush_stat.write_end - _flush_stat.ctrl_start);
}

#else
#define gud_driver_flush_stat_set(m)
static void gud_driver_flush_stat_start() {}
static void gud_driver_flush_stat_print() {}
#endif

static void gud_driver_init(void)
{
    tu_memclr(&_gud_itf, sizeof(_gud_itf));
}

static void gud_driver_reset(uint8_t rhport)
{
    (void) rhport;

//    tu_memclr(&_gud_itf, ITF_MEM_RESET_SIZE);
}

static uint16_t gud_driver_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len)
{
    // GUD_DRV_LOG1("%s:\n", __func__);

    TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass, 0);

    uint16_t const drv_len = sizeof(tusb_desc_interface_t) + itf_desc->bNumEndpoints*sizeof(tusb_desc_endpoint_t);
    TU_VERIFY(max_len >= drv_len, 0);

    tusb_desc_endpoint_t const * desc_ep = (tusb_desc_endpoint_t const *) tu_desc_next(itf_desc);
    TU_ASSERT(usbd_edpt_open(rhport, desc_ep), 0);
    _gud_itf.ep_out = desc_ep->bEndpointAddress;
    _gud_itf.itf_num = itf_desc->bInterfaceNumber;

    return drv_len;
}

static bool gud_driver_control_request(uint8_t rhport, tusb_control_request_t const * req)
{
    uint16_t wLength;
    int ret;

    wLength = min(req->wLength, sizeof(_ctrl_req_buf));

    // GUD_DRV_LOG2("%s:  bRequest=0x%02x bmRequestType=0x%x %s wLength=%u(%u) \n",
    //              __func__, req->bRequest, req->bmRequestType,
    //              req->bmRequestType_bit.direction ? "IN" : "OUT", wLength, req->wLength);

    req_timestamp = esp_timer_get_time();

    if (req->bmRequestType_bit.recipient != TUSB_REQ_RCPT_INTERFACE ||
        req->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR)
        return false;

    if (req->bmRequestType_bit.direction) {

        if (req->bRequest == GUD_REQ_GET_STATUS) {
            // GUD_DRV_LOG2("GUD_REQ_GET_STATUS=%u\n", status);
            return tud_control_xfer(rhport, req, &status, sizeof(status));
        }

        status = 0;
        ret = gud_req_get(_display, req->bRequest, req->wValue, _ctrl_req_buf, wLength);
        if (ret < 0) {
            status = -ret;
            return false;
        }

        return tud_control_xfer(rhport, req, _ctrl_req_buf, ret);
    } else {
        status = 0;

        if (!wLength) {
            int ret = gud_req_set(_display, req->bRequest, req->wValue, _ctrl_req_buf, 0);
            if (ret < 0) {
                status = -ret;
                return false;
            }
        }

        if (req->bRequest == GUD_REQ_SET_BUFFER)
            gud_driver_flush_stat_start();

        return tud_control_xfer(rhport, req, _ctrl_req_buf, wLength);
    }

    return false;
}

static bool gud_driver_bulk_xfer(uint8_t rhport, uint8_t *buf, uint32_t xfer_len, uint32_t len)
{
    TU_ASSERT(!usbd_edpt_busy(rhport, _gud_itf.ep_out));

    if (buf) {
        TU_ASSERT(xfer_len && len);
        _gud_itf.offset = 0;
        _gud_itf.buf = buf;
        _gud_itf.len = len;
        _gud_itf.xfer_len = xfer_len;
    } else {
        _gud_itf.offset += GUD_EDPT_XFER_MAX_SIZE;
        buf = _gud_itf.buf + _gud_itf.offset;
        xfer_len = _gud_itf.xfer_len - _gud_itf.offset;
    }

    if (xfer_len > GUD_EDPT_XFER_MAX_SIZE)
        xfer_len = GUD_EDPT_XFER_MAX_SIZE;

    return usbd_edpt_xfer(rhport, _gud_itf.ep_out, buf, xfer_len);
}

static bool gud_driver_control_complete(uint8_t rhport, tusb_control_request_t const * req)
{
    uint16_t wLength;

    wLength = min(req->wLength, sizeof(_ctrl_req_buf));

    // GUD_DRV_LOG2("%s: bRequest=0x%02x bmRequestType=0x%x %s wLength=%u(%u)\n",
    //              __func__, req->bRequest, req->bmRequestType,
    //              req->bmRequestType_bit.direction ? "IN" : "OUT", wLength, req->wLength);

    if (!req->bmRequestType_bit.direction) {
        int ret = gud_req_set(_display, req->bRequest, req->wValue, _ctrl_req_buf, wLength);
        if (ret < 0) {
            status = -ret;
            return false;
        }

        if (req->bRequest == GUD_REQ_SET_CONTROLLER_ENABLE)
            controller_enabled = *(uint8_t *)_ctrl_req_buf;

        if (req->bRequest == GUD_REQ_SET_BUFFER) {
            const struct gud_set_buffer_req *buf_req = (const struct gud_set_buffer_req *)_ctrl_req_buf;
            uint32_t len;
            void *buf;

            if (buf_req->compression) {
                buf = _compress_buf;
                len = buf_req->compressed_length;
            } else {
                buf = _framebuffer;
                len = buf_req->length;
            }

            gud_driver_flush_stat_set(bulk_start);

            return gud_driver_bulk_xfer(rhport, buf, len, buf_req->length);
        }

        if (req->bRequest == GUD_REQ_SET_STATE_CHECK && _display->flags & GUD_DISPLAY_FLAG_FULL_UPDATE) {
            if (!usbd_edpt_busy(rhport, _gud_itf.ep_out)) {
                const struct gud_state_req *req = (const struct gud_state_req *)_ctrl_req_buf;

                uint32_t len = gud_get_buffer_length(req->format, _display->width, _display->height);
                if (!len) {
                    status = GUD_STATUS_INVALID_PARAMETER;
                    return false;
                }
                return gud_driver_bulk_xfer(rhport, _framebuffer, len, len);
            }
        }
    }

    return true;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * req)
{
    if ( stage == CONTROL_STAGE_SETUP )
        return gud_driver_control_request(rhport, req);
    else if ( stage == CONTROL_STAGE_DATA )
        return gud_driver_control_complete(rhport, req);
    return true;
}

static bool gud_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * req)
{
    return false;
}

static bool gud_driver_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    TU_VERIFY(result == XFER_RESULT_SUCCESS);

    if (xferred_bytes != (_gud_itf.xfer_len - _gud_itf.offset)) {
        if (xferred_bytes != GUD_EDPT_XFER_MAX_SIZE) {
            ESP_LOGI(TAG, "%s: UNHANDLED: xferred_bytes=%"PRIu32" != _gud_itf.xfer_len=%"PRIu32"\n",
                         __func__, xferred_bytes, _gud_itf.xfer_len);
            return false;
        }
        return gud_driver_bulk_xfer(rhport, NULL, 0, 0);
    }

    gud_driver_flush_stat_set(bulk_end);

    if (_gud_itf.xfer_len != _gud_itf.len) {
        // printf("start:%p, end:%p\n", _framebuffer, );
        int ret = LZ4_decompress_safe((const char*)_gud_itf.buf, (char*)_framebuffer, _gud_itf.xfer_len, _gud_itf.len);
        if (ret < 0) {
            ESP_LOGE(TAG, "LZ4_decompress_safe failed %d: xfer_len=%"PRIu32" len=%"PRIu32"\n", ret, _gud_itf.xfer_len, _gud_itf.len);
            return false;
        }

        gud_driver_flush_stat_set(decompress_end);
    }

    gud_write_buffer(_display, _framebuffer);

    gud_driver_flush_stat_set(write_end);
    gud_driver_flush_stat_print();

    if (_display->flags & GUD_DISPLAY_FLAG_FULL_UPDATE){
        return gud_driver_bulk_xfer(rhport, _framebuffer, _gud_itf.xfer_len, _gud_itf.xfer_len);
    }

    return true;
}

/*
 * This can be used to detect loss of communication with the host.
 * I have experienced situations where the host thinks everything is fine,
 * but this driver never sees the request.
 *
 * Set the poll flag to ensure a request is sent every 10 seconds:
 * gud_display.connector_flags = GUD_CONNECTOR_FLAGS_POLL_STATUS
 */
bool gud_driver_req_timeout(unsigned int timeout_secs)
{
    if (!controller_enabled)
        return false;

    uint64_t timeout = req_timestamp + (timeout_secs * 1000 * 1000);
    return esp_timer_get_time() > timeout;
}

static usbd_class_driver_t const _usbd_driver[] =
{
    {
  #if CFG_TUSB_DEBUG >= 2
        .name             = "GUD",
  #endif
        .init             = gud_driver_init,
        .reset            = gud_driver_reset,
        .open             = gud_driver_open,
        .control_xfer_cb  = gud_driver_control_xfer_cb,
        .xfer_cb          = gud_driver_xfer_cb,
        .sof              = NULL
    },
};

usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count)
{
	*driver_count += TU_ARRAY_SIZE(_usbd_driver);

	return _usbd_driver;
}

void gud_driver_setup(const struct gud_display *disp, void *framebuffer, void *compress_buf)
{
    if (disp->compression && !compress_buf)
        printf("Missing compress_buf");

    _display = disp;
    _framebuffer = framebuffer;
    _compress_buf = compress_buf;
}
