//
// Created by felix on 10.11.25.
//


#include "led_driver_task.h"
#include "../t2v_utils.h"
#include "../t2v_address_config.h"

#include <string.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

void translate_logical_to_andreas_routed(
    uint8_t seven_segment_1,
    uint8_t seven_segment_2,
    uint8_t leds,
    uint8_t* data
);

void led_driver_task_main(void* params) {
    LedDriverParams_t* task_params = params;

    // Initialize non-SPI GPIOs
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = 1ULL << CONFIG_T2V_LED_DRIVER_OE_GPIO_NUM;
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);

    // Output is disabled when this is high.
    gpio_set_level(CONFIG_T2V_LED_DRIVER_OE_GPIO_NUM, 1);

    spi_device_handle_t spi;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = CONFIG_T2V_LED_DRIVER_SDI_GPIO_NUM,
        .sclk_io_num = CONFIG_T2V_LED_DRIVER_CLK_GPIO_NUM,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,                              //SPI mode 0
        .spics_io_num = CONFIG_T2V_LED_DRIVER_LE_GPIO_NUM,             //CS pin
        .queue_size = 1,                        //We want to be able to queue 7 transactions at a time
    };

    //Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);
    //Attach the LED to the SPI bus
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

#ifdef CONFIG_T2V_ADDRESS_CONFIG_ENABLE
    uint8_t unicast_address = read_unicast_address();
#endif

    uint8_t data[3];
    uint8_t leds = 0x00;
#ifdef CONFIG_T2V_ADDRESS_CONFIG_ENABLE
    uint8_t seven_segment_1 = nibble_to_seven_segment((unicast_address & 0xf0) >> 4);
    uint8_t seven_segment_2 = nibble_to_seven_segment(unicast_address & 0x0f);
#else
    uint8_t seven_segment_1 = 0x00;
    uint8_t seven_segment_2 = 0x00;
#endif

    translate_logical_to_andreas_routed(
        seven_segment_1,
        seven_segment_2,
        leds,
        data
    );

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
    gpio_set_level(CONFIG_T2V_LED_DRIVER_OE_GPIO_NUM, 0);
    LedUpdate_t update;

    while (1)
    {
        if (xQueueReceive(task_params->in_led_updates, &update, pdMS_TO_TICKS(0)))
        {
            ESP_LOGI(TAG_T2V_MODULE_LED, "update received: flags: %02X, leds: %02X, led_mask: %02X", update.flags, update.leds, update.leds_mask);
            if (update.flags == 0x00)
            {
                continue;
            }

            if (update.flags & LED_UPDATE_SEVEN_SEGMENT_1_VALID)
            {
                seven_segment_1 = update.seven_segment_1;
            }

            if (update.flags & LED_UPDATE_SEVEN_SEGMENT_2_VALID)
            {
                seven_segment_2 = update.seven_segment_2;
            }

            if (update.flags & LED_UPDATE_LEDS_VALID)
            {
                leds = (update.leds & update.leds_mask) | (leds & ~update.leds_mask);
            }
        }

        translate_logical_to_andreas_routed(
            seven_segment_1,
            seven_segment_2,
            leds,
            data
        );

        spi_device_acquire_bus(spi, portMAX_DELAY);

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
    ESP_LOGD(TAG_T2V_MODULE_LED, "seven_segment_1: %02x, seven_segment_2: %02x, leds: %02x", seven_segment_1, seven_segment_2, leds);

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