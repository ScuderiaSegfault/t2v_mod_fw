# T2V Firmware

## Bill of Materials

* [ESP32-P4-NANO](https://www.waveshare.com/esp32-p4-nano.htm)

TODOs:

- [ ] store configuration of addresses in NVS
- [ ] allow address configuration with USB 



## Fix for Cache Invalidation

After the first build (managed components are downloaded during build) modify file `managed_components/espressif__tinyusb/src/portable/synopsys/dwc2/dwc2_esp32.h`.
Add the following method:

```c++
TU_ATTR_ALWAYS_INLINE static inline void* align_to_cache_line(void* address) {
  return (void*)((size_t)address & ~(CONFIG_CACHE_L1_CACHE_LINE_SIZE-1));
}
```

And wrap the address argument of all calls to `esp_cache_msync` in the same file with the new function.
Example:

```c++
return ESP_OK == esp_cache_msync(align_to_cache_line((void*)addr), data_size, flag);
```