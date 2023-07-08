#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "esp_sleep.h"
#include "ulp_main.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

const gpio_num_t DS_PIN = GPIO_NUM_15;

static void ow_write_bit(bool bit) {
    rtc_gpio_set_direction(DS_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
    if (bit) {
        esp_rom_delay_us(2);
        rtc_gpio_set_direction(DS_PIN, RTC_GPIO_MODE_INPUT_ONLY);
        esp_rom_delay_us(120 - 2);
    } else {
        esp_rom_delay_us(120);
        rtc_gpio_set_direction(DS_PIN, RTC_GPIO_MODE_INPUT_ONLY);
    }
}

static bool ow_read_bit() {
    bool result;

    rtc_gpio_set_direction(DS_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
    esp_rom_delay_us(2);
    rtc_gpio_set_direction(DS_PIN, RTC_GPIO_MODE_INPUT_ONLY);
    esp_rom_delay_us(14 - 2);
    result = rtc_gpio_get_level(DS_PIN) != 0;
    esp_rom_delay_us(120 - 14);
    return result;
}

static void ow_write(uint8_t data) {
    for (uint8_t i = 0; i < 8; ++i) {
        ow_write_bit(data & 0x01);
        data >>= 1;
    }
}

static uint8_t ow_read() {
    uint8_t result = 0;

    for (uint8_t i = 0; i < 8; ++i) {
        result >>= 1;
        if (ow_read_bit())
            result |= 0x80;
    }
    return result;
}

static bool ow_reset() {
    bool result;

    rtc_gpio_set_direction(DS_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
    esp_rom_delay_us(480);
    rtc_gpio_set_direction(DS_PIN, RTC_GPIO_MODE_INPUT_ONLY);
    esp_rom_delay_us(70);
    result = rtc_gpio_get_level(DS_PIN) == 0;
    if (result)
        esp_rom_delay_us(480 - 70);
    return result;
}

static uint8_t ow_crc8(const uint8_t *buf, uint8_t size) {
    uint8_t crc = 0;
    uint8_t b;

    while (size--) {
        b = *buf++;
        for (uint8_t i = 8; i > 0; --i) {
            uint8_t mix = (crc ^ b) & 0x01;

            crc >>= 1;
            if (mix)
                crc ^= 0x8C;
            b >>= 1;
        }
    }
    return crc;
}

static bool ds_init() {
    bool result = false;

    rtc_gpio_init(DS_PIN);
    rtc_gpio_set_direction(DS_PIN, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_dis(DS_PIN);
    rtc_gpio_pulldown_dis(DS_PIN);

    if (ow_reset()) {
        uint8_t data[8];

        ow_write(0x33); // READ_ROM
        for (uint8_t i = 0; i < 8; ++i) {
            data[i] = ow_read();
        }
        if ((data[0] == 0x28) && (ow_crc8(data, 7) == data[7])) {
            if (ow_reset()) {
                ow_write(0xCC); // SKIP_ROM
                ow_write(0x44); // CONVERT
                result = true;
            }
        }
    }
    return result;
}

static esp_err_t init_ulp_program() {
    esp_err_t result;

    result = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    if (result == ESP_OK) {
        ulp_set_wakeup_period(0, 750000); // 750 ms.
        result = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    }
    return result;
}

static void halt(const char *msg) {
    Serial.println(msg);
    Serial.flush();
    esp_deep_sleep_start();
}

void setup() {
    Serial.begin(115200);

    if (! ds_init())
        halt("DS1820 not connected!\n");
    if (init_ulp_program() != ESP_OK)
        halt("ULP init fail!");
    Serial.println("DS1820 on ULP demo");
}

void loop() {
    delay(1000);
    Serial.printf("\r%.2f     ", (ulp_temperature & 0xFFFF) / 16.0);
}
