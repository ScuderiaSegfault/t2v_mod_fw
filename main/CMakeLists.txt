idf_component_register(SRCS "firmware_main.c" "ir_nec_encoder.c" "ir_nec_receiver_task.c" "usb_device_task.c"
                       PRIV_REQUIRES esp_driver_rmt esp_tinyusb
                       INCLUDE_DIRS "")
