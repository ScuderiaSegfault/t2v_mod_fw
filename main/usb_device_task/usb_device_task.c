
#include "../t2v.h"
#include "esp_log.h"
#include "tinyusb_default_config.h"
#include "tinyusb.h"
#include "class/vendor/vendor_device.h"
#include "device/usbd_pvt.h"

// 9 = interface descriptor, 7 = endpoint descriptor
#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + 9 + 7)

static const tusb_desc_device_t device_descriptor = {
    .bLength = 18,
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x110,
    .bDeviceClass = TUSB_CLASS_VENDOR_SPECIFIC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x5455,
    .idProduct = 0x1911,
    .bcdDevice = 0x0010,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

#if (TUD_OPT_HIGH_SPEED)
static const tusb_desc_device_qualifier_t device_qualifier_descriptor = {
    .bLength = 10,
    .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB = 0x110,
    .bDeviceClass = TUSB_CLASS_VENDOR_SPECIFIC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .bNumConfigurations = 1,
    .bReserved = 0,
};
#endif

/**
 * @brief String descriptor
 */
const char* string_descriptors[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "TU Wien", // 1: Manufacturer
    "Roboracer T2V Module", // 2: Product
    "123456", // 3: Serials, should use chip ID
    "IR NEC data", // 4: IR NEC data
};

static const uint8_t configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 300),
    9, TUSB_DESC_INTERFACE, 0, 0, 1, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, 4,
    /* Endpoint In */\
    7, TUSB_DESC_ENDPOINT, 0b10000001, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(4), 1
};

/**
 * @brief stores the number of the interface
 */
static uint8_t itf_num;

/**
 * @brief called when the driver is initialized
 */
static void init(void)
{
    ESP_LOGI(TAG_T2V_MODULE_USB, "Initializing USB driver");
}

/**
 * @brief called when the device is reset
 */
static void reset(uint8_t __unused rhport)
{
    itf_num = 0;
}

/**
 * @brief called when the host device is opening this device. Based on the selected interface we also have to open(enable)
 * the relevant endpoints.
 */
static uint16_t open(uint8_t __unused rhport, tusb_desc_interface_t const* itf_desc, uint16_t max_len)
{
    ESP_LOGI(TAG_T2V_MODULE_USB, "Opening USB device");
    // We only have one interface, so make sure that we open the correct one
    TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass &&
              0 == itf_desc->bInterfaceSubClass &&
              0 == itf_desc->bInterfaceProtocol, 0);

    // Verify that the length of the interface matches what we expect
    uint16_t const drv_len = sizeof(tusb_desc_interface_t) + sizeof(tusb_desc_endpoint_t);
    TU_VERIFY(max_len >= drv_len, 0);

    // Reinterpret data as byte pointer and skip forward to the next descriptor
    uint8_t const* p_desc = (uint8_t const*)itf_desc;
    p_desc = tu_desc_next(p_desc);

    // The next descriptor MUST be an endpoint descriptor, so it is safe to interpret it as one
    TU_ASSERT(usbd_edpt_open(rhport, (tusb_desc_endpoint_t const*) p_desc), 0);

    // Store the selected interface number (even if we only define one)
    itf_num = itf_desc->bInterfaceNumber;
    return drv_len;
}

/**
 * @brief not sure when this is called...
 */
static bool control_request_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request)
{
    ESP_LOGI(TAG_T2V_MODULE_USB, "Requested control request");
    return true;
}

/**
 * @brief called when a transfer has finished. Result indicates the result of the transfer
 */
static bool xfer_cb(uint8_t __unused rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    switch (result)
    {
    case XFER_RESULT_SUCCESS:
        ESP_LOGD(TAG_T2V_MODULE_USB, "Transfer on endpoint 0x%02x successful (transferred %d bytes)", ep_addr,
                 xferred_bytes);
        break;
    case XFER_RESULT_FAILED:
        ESP_LOGE(TAG_T2V_MODULE_USB, "Transfer on endpoint 0x%02x failed (transferred %d bytes)", ep_addr,
                 xferred_bytes);
        break;
    case XFER_RESULT_STALLED:
        ESP_LOGE(TAG_T2V_MODULE_USB, "Transfer on endpoint 0x%02x failed: stalled (transferred %d bytes)", ep_addr,
                 xferred_bytes);
        break;
    case XFER_RESULT_TIMEOUT:
        ESP_LOGE(TAG_T2V_MODULE_USB, "Transfer on endpoint 0x%02x timed out (transferred %d bytes)", ep_addr,
                 xferred_bytes);
        break;
    case XFER_RESULT_INVALID:
        ESP_LOGE(TAG_T2V_MODULE_USB, "Transfer on endpoint 0x%02x invalid (transferred %d bytes)", ep_addr,
                 xferred_bytes);
        break;
    }
    return true;
}

/**
 * @brief the driver is defined by the functions in the struct
 */
static usbd_class_driver_t const app_driver =
{
#if CFG_TUSB_DEBUG >= 2
    .name = "T2V_MODULE",
#endif
    .init = init,
    .reset = reset,
    .open = open,
    .control_xfer_cb = control_request_cb,
    .xfer_cb = xfer_cb,
    .sof = NULL
};

/**
 * @brief inform the stack about our custom driver
 */
usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count)
{
    // we only add one additional driver
    *driver_count = 1;
    return &app_driver; // return the address of the custom driver
}

/**
 * @brief TinyUSB callback for device event handler
 *
 * @note
 * For Linux-based Hosts: Reflects the SetConfiguration() request from the Host Driver.
 * For Win-based Hosts: SetConfiguration() request is present only with available Class in device descriptor.
 */
void device_event_handler(tinyusb_event_t* event, __unused void* arg)
{
    switch (event->id)
    {
    case TINYUSB_EVENT_ATTACHED:
        // Device has been attached to the USB Host and configured
        ESP_LOGI(TAG_T2V_MODULE_USB, "Device attached");
        break;
    case TINYUSB_EVENT_DETACHED:
        // Device has been detached from the USB Host
        ESP_LOGI(TAG_T2V_MODULE_USB, "Device detached");
        break;
    default:
        break;
    }
}

void usb_device_task_main(void* params)
{
    UsbDeviceTaskParams_t* task_params = params;

    ESP_LOGI(TAG_T2V_MODULE_USB, "initializing USB stack");
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(device_event_handler);
    tusb_cfg.port = TINYUSB_PORT_FULL_SPEED_0;
    tusb_cfg.descriptor.device = &device_descriptor;
    tusb_cfg.descriptor.string = string_descriptors;
    tusb_cfg.descriptor.string_count = sizeof(string_descriptors) / sizeof(string_descriptors[0]);
    tusb_cfg.descriptor.full_speed_config = configuration_descriptor;
#if (TUD_OPT_HIGH_SPEED)
    tusb_cfg.descriptor.high_speed_config = configuration_descriptor;
    tusb_cfg.descriptor.qualifier = &device_qualifier_descriptor;
#endif

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG_T2V_MODULE_USB, "finished initializing USB stack");
    uint8_t data[4];

    while (1)
    {
        if (xQueueReceive(task_params->in_ir_data, &data, pdMS_TO_TICKS(10)))
        {
            if (tud_mounted())
            {
                usbd_edpt_xfer(
                    0,
                    0x81,
                    data,
                    4
                );
            }
        }
    }
}
