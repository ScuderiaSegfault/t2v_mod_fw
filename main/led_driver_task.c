//
// Created by felix on 10.11.25.
//


#include "common.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

void translate_logical_to_andreas_routed(
    const uint8_t seven_segment_1,
    const uint8_t seven_segment_2,
    const uint8_t leds,
    uint8_t* data
);

void led_driver_task_main(void *) {
    // Initialize non-SPI GPIOs
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = 1ULL << LED_DRIVER_OE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);

    // Output is disabled when this is high.
    gpio_set_level(LED_DRIVER_OE, 1);

    spi_device_handle_t spi;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = LED_DRIVER_SDI,
        .sclk_io_num = LED_DRIVER_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,     //Clock out at 30 MHz
        .mode = 0,                              //SPI mode 0
        .spics_io_num = LED_DRIVER_LE,             //CS pin
        .queue_size = 1,                        //We want to be able to queue 7 transactions at a time
    };

    //Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);
    //Attach the LED to the SPI bus
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    uint8_t data[3] = {0};

    spi_device_acquire_bus(spi, portMAX_DELAY);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * 3;
    t.flags = 0;
    t.user = (void*) data;

    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);

    // Release bus
    spi_device_release_bus(spi);

    // Output is disabled when this is high.
    gpio_set_level(LED_DRIVER_OE, 0);

    uint8_t leds = 0x01;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(250));

        leds = leds << 1;
        if (leds == 0x00) {
            leds = 0x01;
        }

        translate_logical_to_andreas_routed(
            leds,
            leds,
            leds,
            data
        );

        spi_device_acquire_bus(spi, portMAX_DELAY);

        ESP_LOG_BUFFER_HEX(TAG_T2V_MODULE_LED, data, 3);

        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = 8 * 3;
        t.flags = 0;
        t.tx_buffer = data;

        ret = spi_device_polling_transmit(spi, &t);
        assert(ret == ESP_OK);

        // Release bus
        spi_device_release_bus(spi);
    }
}

uint8_t nibble_to_seven_segment(uint8_t nibble)
{
    switch (nibble & 0x0f)
    {
        case 0x0:
            return 0b00111111;
        case 0x1:
            return 0b00000110;
        case 0x2:
            return 0b11011011;
        case 0x3:
            return 0b11001111;
        case 0x4:
            return 0b01100110;
        case 0x5:
            return 0b11101101;
        case 0x6:
            return 0b11111101;
        case 0x7:
            return 0b00000111;
        case 0x8:
            return 0xef;
        case 0x9:
            return 0b11101111;
        case 0xa:
            return 0b01110111;
        case 0xb:
            return 0b11111100;
        case 0xc:
            return 0b00111001;
        case 0xd:
            return 0b11011110;
        case 0xe:
            return 0b11111001;
        case 0xf:
            return 0b01110001;
        default:
            return 0x00;
    }
}


/**
 * This method translates between a somewhat logical representation of the desired display value to whatever Andreas has
 * routed on the PCB...
 *
 * The mappings between bytes to seven segments are as follows:
 *
 * Bitmap of the value: hgfedcba
 *
 * The bitmap is mapped to the 7-segment display as follows:
 * /-a-\
 * f   b
 * |-g-|
 * e   c
 * \-d-/   h
 * (if this doesn't look like a seven segment display to you, I completely understand...)
 *
 * The LEDs are mapped like this:
 *
 * 0x01: Red front
 * 0x02: Yellow front
 * 0x04: Green front
 * 0x08: Blue front
 * 0x10: Red rear
 * 0x20: Yellow rear
 * 0x40: Green rear
 * 0x80: Blue rear
 *
 * @param seven_segment_1
 * @param seven_segment_2
 * @param leds
 * @param data
 */
void translate_logical_to_andreas_routed(
    const uint8_t seven_segment_1,
    const uint8_t seven_segment_2,
    const uint8_t leds,
    uint8_t* data
)
{
    ESP_LOGI(TAG_T2V_MODULE_LED, "seven_segment_1: %02x, seven_segment_2: %02x, leds: %02x", seven_segment_1, seven_segment_2, leds);

    data[0] = 0;
    data[1] = 0;
    data[2] = 0;

    data[0] |= (leds & 0x80) >> 7;
    data[0] |= (leds & 0x40) >> 5;
    data[0] |= (leds & 0x20) >> 3;
    data[0] |= (leds & 0x10) >> 1;
    data[1] |= (leds & 0x0f) << 4;

    data[2] |= (seven_segment_1 & 0x01) << 1;
    data[2] |= (seven_segment_1 & 0x02) >> 1;
    data[0] |= (seven_segment_1 & 0x04) << 3;
    data[0] |= (seven_segment_1 & 0x08) << 1;
    data[2] |= (seven_segment_1 & 0x10) << 3;
    data[2] |= (seven_segment_1 & 0x20) >> 3;
    data[2] |= (seven_segment_1 & 0x40) >> 3;
    data[0] |= (seven_segment_1 & 0x80) >> 1;

    data[2] |= (seven_segment_2 & 0x01) << 6;
    data[1] |= (seven_segment_2 & 0x02) << 2;
    data[1] |= seven_segment_2 & 0x04;
    data[1] |= (seven_segment_2 & 0x08) >> 2;
    data[1] |= (seven_segment_2 & 0x10) >> 4;
    data[2] |= seven_segment_2 & 0x20;
    data[2] |= (seven_segment_2 & 0x40) >> 2;
    data[0] |= seven_segment_2 & 0x80;

}