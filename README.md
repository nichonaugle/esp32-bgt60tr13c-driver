# Espressif BGT60TR13C Driver

The BGT60TR13C is a powerful radar sensor featuring 3 RX and 1 TX antenna, but its setup and usage can be complex. This driver simplifies the integration process, allowing developers to focus on signal processing algorithms rather than low-level interfacing. It manages all register control and is designed to capture three 64×32 frames, returning 12-bit words in a 16-bit array.

Additionally, included Python scripts enable the generation of different frame sizes. However, modifying some header files may be necessary to accommodate these changes.

## Installation

1. Clone the repository:
```
git clone https://github.com/your-username/esp32-bgt60tr13c-driver.git
```

2. Add the driver to your ESP-IDF project:
   - Copy the `bgt60tr13c_driver.c` and `bgt60tr13c_driver.h` files to your project's `components` directory.
   - Update your project's `CMakeLists.txt` or `Makefile` to include the `bgt60tr13c_driver` component.

3. Configure the SPI bus and radar settings in your application code. Refer to the [Usage](#usage) section for an example.

## Usage

1. Initialize the SPI bus and the BGT60TR13C radar driver:

```c
// Recommended SPI bus configuration
spi_bus_config_t bus_config = {
    .miso_io_num = SPI_MISO_PIN,
    .mosi_io_num = SPI_MOSI_PIN,
    .sclk_io_num = SPI_SCK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 32
};

esp_err_t ret = spi_bus_initialize(SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
    return;
}

// Customize SPI bus for the radar
spi_device_interface_config_t dev_config = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .clock_speed_hz = SPI_CLK_SPEED * 1000 * 1000,
    .mode = 0,
    .spics_io_num = SPI_CS_PIN,
    .queue_size = 1,
};

ret = xensiv_bgt60tr13c_init(SPI_HOST, &dev_config);
assert(ret == ESP_OK);
```

2. Capture radar frames:

```c
// Collects one frame when triggered
uint32_t frame_size = 0;
uint32_t irq_frame_size = 0;
ESP_ERROR_CHECK(get_frame_size(&frame_size));
ESP_ERROR_CHECK(get_interrupt_frame_size_trigger(&irq_frame_size));

uint32_t frame_buf_len = frame_size;
uint16_t *frame_buf = (uint16_t *)malloc(frame_buf_len * sizeof(uint16_t));

ESP_ERROR_CHECK(xensiv_bgt60tr13c_start_frame_capture());

// Collect frames until complete
for (;;) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        xensiv_bgt60tr13c_fifo_read(temp_buf, temp_buf_len, irq_frame_size);
        // Process the collected frame data
        // ...
    }
}
```

## API

The `bgt60tr13c_driver.h` file provides the following API functions:

- `xensiv_bgt60tr13c_init`: Initializes the BGT60TR13C radar driver and attaches the device to the SPI bus.
- `xensiv_bgt60tr13c_configure`: Configures the radar registers based on the settings defined in `bgt60tr13c_config.h`.
- `xensiv_bgt60tr13c_start_frame_capture`: Starts the radar frame capture process.
- `xensiv_bgt60tr13c_fifo_read`: Reads the radar frame data from the FIFO.
- `xensiv_bgt60tr13c_set_reg`: Sets a specific register in the radar.
- `xensiv_bgt60tr13c_get_reg`: Reads the value of a specific register in the radar.
- `xensiv_bgt60tr13c_soft_reset`: Performs a soft reset of the radar.
- `get_frame_size`: Retrieves the size of the radar frame.
- `get_interrupt_frame_size_trigger`: Retrieves the size of the radar frame that triggers an interrupt.

## Contributing

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Implement your changes and ensure the code passes all tests.
4. Submit a pull request with a detailed description of your changes.

## License

This project is licensed under the [MIT License](LICENSE).

## Testing

To run the tests for the BGT60TR13C driver, follow these steps:

1. Ensure that the necessary hardware (BGT60TR13C radar and ESP32 board) is connected and configured correctly.
2. Modify the `test.c` file to match your hardware setup (e.g., GPIO pin assignments).
3. Build and flash the test application to your ESP32 board.
4. Monitor the serial output to observe the test results.

The `test.c` file provides an example of how to use the BGT60TR13C driver in an application and perform basic functionality testing.
