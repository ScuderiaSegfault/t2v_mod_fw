
list(APPEND T2V_SRCS "t2v_init.c")

if(CONFIG_T2V_BLE_ENABLE)
    list(APPEND T2V_SRCS "ble_task/ble_task.c")
endif()

if(CONFIG_T2V_BTN_INPUT_ENABLE)
    list(APPEND T2V_SRCS "btn_input_task/btn_input_task.c")
endif()

if(CONFIG_T2V_ETHERNET_ENABLE)
    list(APPEND T2V_SRCS "ethernet_task/ethernet_task.c")
endif()

if(CONFIG_T2V_IR_NEC_RX_ENABLE)
    list(APPEND T2V_SRCS "ir_nec_rx_task/ir_nec_rx_task.c")
endif()

if(CONFIG_T2V_IR_NEC_TX_ENABLE)
    list(APPEND T2V_SRCS "ir_nec_tx_task/ir_nec_encoder.c" "ir_nec_tx_task/ir_nec_tx_task.c")
endif()

if(CONFIG_T2V_LED_DRIVER_ENABLE)
    list(APPEND T2V_SRCS "led_driver_task/led_driver_task.c")
endif()

if(CONFIG_T2V_USB_DEVICE_ENABLE)
    list(APPEND T2V_SRCS "usb_device_task/usb_device_task.c")
endif()

if(CONFIG_T2V_WS2812_ENABLE)
    list(APPEND T2V_SRCS "ws2812_task/led_strip_encoder.c" "ws2812_task/ws2812_task.c")
endif()

if(CONFIG_T2V_ZENOH_ENABLE)
    list(APPEND T2V_SRCS "zenoh_task/zenoh_task.c")
endif()

message("src files: ${T2V_SRCS}")

idf_component_register(SRCS
        ${T2V_SRCS}
        PRIV_REQUIRES
        bt
        nvs_flash
        esp_hosted
        esp_driver_rmt
        esp_driver_spi
        esp_driver_gpio
        esp_tinyusb
        esp_eth
        ethernet_init
        zenoh-pico
        esp_driver_uart
        INCLUDE_DIRS
        ""
)

add_definitions(-DZENOHPICO_PLATFORM_ESP32)
add_definitions(-DZENOH_ESPIDF=1)