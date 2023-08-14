#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include <errno.h>
#include <dirent.h>
// #include "esp_console.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "esp_private/usb_phy.h"
#include "tusb.h"
#include "driver.h"
#include "gud.h"
#include "display.h"

#define BL_DEF_LEVEL    100
#define WIDTH   320
#define HEIGHT  240

// There's not room for to full buffers so max_buffer_size must be set
uint16_t framebuffer[WIDTH * HEIGHT];
uint16_t compress_buf[WIDTH * 60];


static const uint8_t pixel_formats[] = {
    GUD_PIXEL_FORMAT_RGB565,
};

static const struct gud_property_req connector_properties[] = {
    {
        .prop = GUD_PROPERTY_BACKLIGHT_BRIGHTNESS,
        .val = BL_DEF_LEVEL,
    },
};

TFT_t lcd = {
    .sck = 7,
    .mosi = 6,
    .cs = 11,
    .dc = 12,
    .reset = 47,
    .bl = 48,
    .baudrate = 64 * 1024 * 1024,
    .width = WIDTH,
    .height = HEIGHT
};

tusb_desc_device_t const device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB          	= 0x0110, // USB 1.1 device

    .bDeviceClass    	= 0,      // Specified in interface descriptor
    .bDeviceSubClass 	= 0,      // No subclass
    .bDeviceProtocol 	= 0,      // No protocol
    .bMaxPacketSize0 	= 64,     // Max packet size for ep0

    .idVendor        	= 0x16d0,
    .idProduct       	= 0x10a9,
    .bcdDevice       	= 0,      // Device revision number

    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber 	    = 3,

    .bNumConfigurations = 1
};

typedef struct TU_ATTR_PACKED {
    tusb_desc_configuration_t config;
    tusb_desc_interface_t interface;
    tusb_desc_endpoint_t bulk_out;
} gud_display_config_descriptor_t;

gud_display_config_descriptor_t config_descriptor = {
    .config = {
        .bLength = sizeof(tusb_desc_configuration_t),
        .bDescriptorType = TUSB_DESC_CONFIGURATION,
        .wTotalLength = sizeof(gud_display_config_descriptor_t),
        .bNumInterfaces = 1,
        .bConfigurationValue = 1,
        .iConfiguration = 0,
        .bmAttributes = TU_BIT(7) | TUSB_DESC_CONFIG_ATT_SELF_POWERED,
        .bMaxPower = 100 / 2,
    },

    .interface = {
        .bLength = sizeof(tusb_desc_interface_t),
        .bDescriptorType = TUSB_DESC_INTERFACE,
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 1,
        .bInterfaceClass = TUSB_CLASS_VENDOR_SPECIFIC,
        .bInterfaceSubClass = 0x00,
        .bInterfaceProtocol = 0x00,
        .iInterface = 0,
    },

    .bulk_out = {
        .bLength = sizeof(tusb_desc_endpoint_t),
        .bDescriptorType = TUSB_DESC_ENDPOINT,
        .bEndpointAddress = 3,
        .bmAttributes = {TUSB_XFER_BULK},
        .wMaxPacketSize = 64,
    },
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
    (void) index;
    return (uint8_t const *)&config_descriptor;
}

typedef struct TU_ATTR_PACKED
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t unicode_string[31];
} gud_desc_string_t;

static gud_desc_string_t string_descriptor = {
    .bDescriptorType = TUSB_DESC_STRING,
};

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void) langid;

    if (index == 0) {
        string_descriptor.bLength = 4;
        string_descriptor.unicode_string[0] = 0x0409;
        return (uint16_t *)&string_descriptor;
    }

    const char *str;
    // char serial[17];

    if (index == 1) {
        str = "ESP32 S3";
    } else if (index == 2) {
        str = "ESP32 GUD Display";
    } else if (index == 3) {
        str = "serial number";
    } else {
        return NULL;
    }

    uint8_t len = strlen(str);
    if (len > sizeof(string_descriptor.unicode_string))
        len = sizeof(string_descriptor.unicode_string);

    string_descriptor.bLength = 2 + 2 * len;

    for (uint8_t i = 0; i < len; i++)
      string_descriptor.unicode_string[i] = str[i];

    return (uint16_t *)&string_descriptor;
}

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *) &device_descriptor;
}

static usb_phy_handle_t phy_hdl;

// Configure USB PHY
usb_phy_config_t phy_conf = {
    .controller = USB_PHY_CTRL_OTG,
    .otg_mode = USB_OTG_MODE_DEVICE,
    .target = USB_PHY_TARGET_INT,
};

static int controller_enable(const struct gud_display *disp, uint8_t enable)
{
    // Do nothing 
    return 0;
}

static int display_enable(const struct gud_display *disp, uint8_t enable)
{
    // Turn on or off backlight
    return 0;
}

static int state_commit(const struct gud_display *disp, const struct gud_state_req *state, uint8_t num_properties)
{
    // Used for changing settings (brightness)
    return 0;
}

static int set_buffer(const struct gud_display *disp, const struct gud_set_buffer_req *set_buf)
{
    // Wait for SPI transfer to finish
    return 0;
}

static void write_buffer(const struct gud_display *disp, const struct gud_set_buffer_req *set_buf, void *buf)
{
    // Write buffer to particular display window
    uint32_t length = set_buf->length;

    mipi_dbi_update16(&lcd, set_buf->x, set_buf->y, set_buf->width, set_buf->height, buf, length);

}

static uint32_t gud_display_edid_get_serial_number(void)
{
    uint8_t id_out[8] = {1, 2, 3, 4, 5, 6, 7, 8};

    return *((uint64_t*)(id_out));
}

static const struct gud_display_edid edid = {
    .name = "RPi-Display",
    .pnp = "WAT",
    .product_code = 0x01,
    .year = 2021,
    .width_mm = 58,
    .height_mm = 43,

    .get_serial_number = gud_display_edid_get_serial_number,
};

const struct gud_display disp = {
    .width = WIDTH,
    .height = HEIGHT,

    // .flags = GUD_DISPLAY_FLAG_FULL_UPDATE,

    .compression = GUD_COMPRESSION_LZ4,
    .max_buffer_size = sizeof(compress_buf),
    // .max_buffer_size = WIDTH * 120 * sizeof(uint16_t),

    .formats = pixel_formats,
    .num_formats = 1,

    .connector_properties = connector_properties,
    .num_connector_properties = 1,

    .edid = &edid,

    .controller_enable = controller_enable,
    .display_enable = display_enable,

    .state_commit = state_commit,

    .set_buffer = set_buffer,
    .write_buffer = write_buffer,
};


void app_main(void)
{
  
  usb_new_phy(&phy_conf, &phy_hdl);

  init_display(&lcd);

  gud_driver_setup(&disp, framebuffer, compress_buf);

  tusb_init();

   while (1)
    {
    //   printf("start: %p, end: %p\n", compress_buf[0], compress_buf[(WIDTH * 120) - 1]);
      tud_task(); // tinyusb device task
    }

}
