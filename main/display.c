#include "display.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "esp_timer.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

#define TAG "DISPLAY"

#define TFT_ID SPI2_HOST

#ifndef BIT
  #define BIT(nr)	(1u << (nr))
#endif

#define ILI9341_FRMCTR1         0xb1
#define ILI9341_DISCTRL         0xb6
#define ILI9341_ETMOD           0xb7

#define ILI9341_PWCTRL1         0xc0
#define ILI9341_PWCTRL2         0xc1
#define ILI9341_VMCTRL1         0xc5
#define ILI9341_VMCTRL2         0xc7
#define ILI9341_PWCTRLA         0xcb
#define ILI9341_PWCTRLB         0xcf

#define ILI9341_PGAMCTRL        0xe0
#define ILI9341_NGAMCTRL        0xe1
#define ILI9341_DTCTRLA         0xe8
#define ILI9341_DTCTRLB         0xea
#define ILI9341_PWRSEQ          0xed

#define ILI9341_EN3GAM          0xf2
#define ILI9341_PUMPCTRL        0xf7

#define ILI9341_MADCTL_BGR      BIT(3)
#define ILI9341_MADCTL_MV       BIT(5)
#define ILI9341_MADCTL_MX       BIT(6)
#define ILI9341_MADCTL_MY       BIT(7)

static const int SPI_Command_Mode = 0;
static const int SPI_Data_Mode = 1;
static const int TFT_Frequency = SPI_MASTER_FREQ_40M;

void spi_init(TFT_t * device){
  esp_err_t ret;

  gpio_reset_pin((gpio_num_t)device->cs);
	gpio_set_direction((gpio_num_t)device->cs, GPIO_MODE_OUTPUT );
	gpio_set_level((gpio_num_t)device->cs, 1);

	gpio_reset_pin((gpio_num_t)device->dc);
	gpio_set_direction((gpio_num_t)device->dc, GPIO_MODE_OUTPUT );
	gpio_set_level((gpio_num_t)device->dc, 0);

  gpio_reset_pin((gpio_num_t)device->reset);
  gpio_set_direction((gpio_num_t)device->reset, GPIO_MODE_OUTPUT );
  gpio_set_level((gpio_num_t)device->reset, 0 );
  vTaskDelay(100/ portTICK_PERIOD_MS);
  gpio_set_level((gpio_num_t)device->reset, 1 );

  spi_bus_config_t tft_buscfg = {
		.sclk_io_num = device->sck,
		.mosi_io_num = device->mosi,
		.miso_io_num = -1,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};
	
	spi_device_interface_config_t tft_devcfg={
		.clock_speed_hz = TFT_Frequency,
		.mode=0,
		.spics_io_num = device->cs,
		.queue_size = 7,
		.flags = SPI_DEVICE_NO_DUMMY
	};

  ret = spi_bus_initialize( TFT_ID, &tft_buscfg, SPI_DMA_CH_AUTO );
  ESP_LOGI(TAG, "spi_bus_initialize(TFT) ret=%d TFT_ID=%d",ret, TFT_ID);
	assert(ret==ESP_OK);

  spi_device_handle_t tft_handle;
	ret = spi_bus_add_device( TFT_ID, &tft_devcfg, &tft_handle);
	ESP_LOGD(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);

	device->spi = tft_handle;

}

bool spi_write_bytes(spi_device_handle_t SPIHandle, const uint8_t* Data, size_t DataLength){

	spi_transaction_t SPITransaction;
	esp_err_t ret;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Data;
		ret = spi_device_transmit( SPIHandle, &SPITransaction );
		// ret = spi_device_polling_transmit( SPIHandle, &SPITransaction );

		assert(ret==ESP_OK); 
	}

	return true;
}

// Not enough space in ram for this do its updated 
// bool spi_write_words(spi_device_handle_t SPIHandle, const uint16_t* Data, size_t DataLength){
// 	static uint8_t Byte[153600];
// 	int index = 0;
// 	for(int i=0;i<DataLength;i++) {
// 		Byte[index++] = (Data[i] >> 8) & 0xFF;
// 		Byte[index++] = Data[i] & 0xFF;
// 	}
// 	return spi_write_bytes(SPIHandle, Byte, DataLength*2);
// }

#define MAX_PACKET_SIZE 4092 // Adjust this value as needed based on spi buffer size

bool spi_write_words(spi_device_handle_t SPIHandle, const uint16_t* Data, size_t DataLength) {
    static uint8_t Byte[MAX_PACKET_SIZE];
    
    for (size_t i = 0; i < DataLength; i += MAX_PACKET_SIZE / 2) {
        size_t chunkSize = DataLength - i;
        if (chunkSize > MAX_PACKET_SIZE / 2) {
            chunkSize = MAX_PACKET_SIZE / 2;
        }

        int index = 0;
        for (size_t j = 0; j < chunkSize; j++) {
            Byte[index++] = (Data[i + j] >> 8) & 0xFF;
            Byte[index++] = Data[i + j] & 0xFF;
        }

        if (!spi_write_bytes(SPIHandle, Byte, chunkSize * 2)) {
            // Return false if the transmission fails for any chunk
            return false;
        }
    }

    return true; // All chunks transmitted successfully
}


void mipi_dbi_command_buf(TFT_t * device, uint8_t cmd, const uint8_t *data, size_t len){
    gpio_set_level((gpio_num_t)device->dc, SPI_Command_Mode);
    spi_write_bytes(device->spi, &cmd, 1);

    if (len) {
      gpio_set_level((gpio_num_t)device->dc, SPI_Data_Mode);
      spi_write_bytes(device->spi, data, len);
    }
}

void mipi_dbi_update16(TFT_t * device, uint16_t x, uint16_t y, uint16_t width, uint16_t height, void *buf, size_t len)
{
  set_display_window(device, x, y, width, height);

  gpio_set_level((gpio_num_t)device->dc, SPI_Command_Mode);
  uint8_t cmd = MIPI_DCS_WRITE_MEMORY_START;
  spi_write_bytes(device->spi, &cmd, 1);

  gpio_set_level((gpio_num_t)device->dc, SPI_Data_Mode);

  spi_write_words(device->spi, buf, len / 2);

}

void mipi_dbi_update_wait(TFT_t * device){
	// used to wait till dma transfer is finished 
}

void mipi_dbi_hw_reset(uint8_t gpio){
	gpio_set_level((gpio_num_t)gpio, 0);
	vTaskDelay(20/ portTICK_PERIOD_MS);
	gpio_set_level((gpio_num_t)gpio, 1);
	vTaskDelay(100/ portTICK_PERIOD_MS);
}

void set_display_window(TFT_t * device, uint16_t x, uint16_t y,  uint16_t width, uint16_t height)
{
	uint16_t xe = x + width - 1;
	uint16_t ye = y + height - 1;

	mipi_dbi_command(device, MIPI_DCS_SET_COLUMN_ADDRESS,
										x >> 8, x & 0xff, xe >> 8, xe & 0xff);
	mipi_dbi_command(device, MIPI_DCS_SET_PAGE_ADDRESS,
										y >> 8, y & 0xff, ye >> 8, ye & 0xff);
}

void init_display(TFT_t * device){

	gpio_reset_pin(device->bl);
	gpio_set_direction(device->bl, GPIO_MODE_OUTPUT );
	gpio_set_level(device->bl, 1);

	spi_init(device);

	mipi_dbi_command(device, MIPI_DCS_SOFT_RESET);

	vTaskDelay(150/ portTICK_PERIOD_MS);

	mipi_dbi_command(device, MIPI_DCS_SET_DISPLAY_OFF);

	mipi_dbi_command(device, ILI9341_PWCTRLB, 0x00, 0x83, 0x30);
	mipi_dbi_command(device, ILI9341_PWRSEQ, 0x64, 0x03, 0x12, 0x81);
	mipi_dbi_command(device, ILI9341_DTCTRLA, 0x85, 0x01, 0x79);
	mipi_dbi_command(device, ILI9341_PWCTRLA, 0x39, 0x2c, 0x00, 0x34, 0x02);
	mipi_dbi_command(device, ILI9341_PUMPCTRL, 0x20);
	mipi_dbi_command(device, ILI9341_DTCTRLB, 0x00, 0x00);

	/* Power Control */
	mipi_dbi_command(device, ILI9341_PWCTRL1, 0x26);
	mipi_dbi_command(device, ILI9341_PWCTRL2, 0x11);
	/* VCOM */
	mipi_dbi_command(device, ILI9341_VMCTRL1, 0x35, 0x3e);
	mipi_dbi_command(device, ILI9341_VMCTRL2, 0xbe);

	/* Memory Access Control */
	mipi_dbi_command(device, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FORMAT_16BIT);

	/* Frame Rate */
	mipi_dbi_command(device, ILI9341_FRMCTR1, 0x00, 0x1b);

	/* Gamma */
	mipi_dbi_command(device, ILI9341_EN3GAM, 0x08);
	mipi_dbi_command(device, MIPI_DCS_SET_GAMMA_CURVE, 0x01);
	mipi_dbi_command(device, ILI9341_PGAMCTRL,
										0x1f, 0x1a, 0x18, 0x0a, 0x0f, 0x06, 0x45, 0x87,
										0x32, 0x0a, 0x07, 0x02, 0x07, 0x05, 0x00);
	mipi_dbi_command(device, ILI9341_NGAMCTRL,
										0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3a, 0x78,
										0x4d, 0x05, 0x18, 0x0d, 0x38, 0x3a, 0x1f);

	/* DDRAM */
	mipi_dbi_command(device, ILI9341_ETMOD, 0x07);

	/* Display */
	mipi_dbi_command(device, ILI9341_DISCTRL, 0x0a, 0x82, 0x27, 0x00);
	mipi_dbi_command(device, MIPI_DCS_EXIT_SLEEP_MODE);
	
	vTaskDelay(100/ portTICK_PERIOD_MS);

	uint16_t rotation = 180;
	uint8_t addr_mode;

	switch (rotation) {
	default:
			addr_mode = ILI9341_MADCTL_MV | ILI9341_MADCTL_MY |
							ILI9341_MADCTL_MX;
			break;
	case 90:
			addr_mode = ILI9341_MADCTL_MY;
			break;
	case 180:
			addr_mode = ILI9341_MADCTL_MV;
			break;
	case 270:
			addr_mode = ILI9341_MADCTL_MX;
			break;
	}
	addr_mode |= ILI9341_MADCTL_BGR;
	mipi_dbi_command(device, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);


	mipi_dbi_command(device, MIPI_DCS_SET_DISPLAY_ON);
	vTaskDelay(100/ portTICK_PERIOD_MS);

	// Clear display
	set_display_window(device, 0, 0, 0, 0);

  gpio_set_level((gpio_num_t)device->dc, SPI_Command_Mode);
  uint8_t cmd = MIPI_DCS_WRITE_MEMORY_START;
  spi_write_bytes(device->spi, &cmd, 1);

  gpio_set_level((gpio_num_t)device->dc, SPI_Data_Mode);

  // spi_write_words(device->spi, buf, len / 2);

	uint16_t framebuffer[320];
	for (int i=0;i<320;i++) {
		framebuffer[i] = 0xFAFA;
	}

	for(int i=0;i<=240;i++) {
		uint16_t size = 320;
		spi_write_words(device->spi, framebuffer, size);
	}
	// vTaskDelay(1000/ portTICK_PERIOD_MS);
}