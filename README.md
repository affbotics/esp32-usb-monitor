# esp32-usb-monitor

This project implements a GUD (Graphics USB Display) driver for the ESP32-S3 microcontroller from Espressif. The code has been ported from the original repository [notro/gud-pico](https://github.com/notro/gud-pico/tree/main).

## Getting Started

Follow these steps to set up and use the code:

1. Install ESP-IDF v5.1 by referring to the [official ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).

2. Optionally, you can install Visual Studio Code (VSCode) for a more convenient development experience.

3. Clone or download this repository to your local machine.

4. Set up the Environment Variables for ESP-IDF.

5. Set the IDF target to ESP32-S3 by running the following command:
   ```
   idf.py set-target esp32s3
   ```

6. Build the code using the following command:
   ```
   idf.py build
   ```

7. Flash the program to your ESP32-S3:
   ```
   idf.py flash
   ```

8. Connect the ESP32-S3 to an **ili9340-based** TFT LCD display using the provided pinout, you can use ESP32-S3 DevKitC 1:


| TFT LCD Connection | ESP32-S3 GPIO |
|--------------------|--------------|
| SCK                | GPIO 7       |
| SDI (MOSI)         | GPIO 6       |
| CS                 | GPIO 11      |
| DC                 | GPIO 12      |
| Reset              | GPIO 47      |
| Backlight          | GPIO 48      |
| USB D-             | GPIO 19      |
| USB D+             | GPIO 20      |


9. Connect the ESP32-S3 to a PC's USB port with a GUD driver using GPIO 19 (D-) and GPIO 20 (D+). The ESP32-S3 should show up as a GUD USB display.

## Known Bug

There is a known bug in this code: the device may crash when there is a lot of content on the screen.

Feel free to explore, modify, and improve the code to suit your needs. If you encounter any issues or have suggestions for enhancements, please create an issue on this repository.

Happy coding! 🚀
