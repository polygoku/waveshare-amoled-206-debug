/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Waveshare ESP32-S3-Touch-AMOLED-2.06:
 * https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-2.06
 * https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-2.06
 * ESP32-S3R8 (8 MB octal PSRAM), 32 MB flash, 2.06" 410x502 CO5300 AMOLED
 * over QSPI @ 80 MHz, FT3168 capacitive touch (0x38), AXP2101 PMU (0x34),
 * ES8311 speaker codec, ES7210 dual-microphone ADC, QMI8658 IMU and
 * PCF85063 RTC — all controlled over the shared BSP I2C bus.
 * BOOT is GPIO0. Native USB Serial/JTAG (303a:1001).
 *
 * Pin/fact sources (per devices/AGENTS.md, do not guess pins):
 *  - Vendor wiki pinout: AMOLED QSPI SDIO0..3 = GPIO 4/5/6/7, SCLK = 11,
 *    CS = 12, RST = 8 (needs a LONG pulse: 200 ms low — short pulses don't
 *    boot the panel); I2C0 SDA = 15, SCL = 14; touch FT3168 RST = 9,
 *    INT = 38 (needs reg 0xA5 = monitor mode at init); audio I2S
 *    DOUT = 40, BCLK = 41, LRC = 45, MCLK = 16, PA enable = 46 (high only
 *    while playing audio); ES7210 microphone ADC ASDOUT = 42.
 *  - Managed BSP: waveshare/esp32_s3_touch_amoled_2_06 ^2.0.0 (ESP Component
 *    Registry), which drives the CO5300 panel, FT3168 touch, ES8311
 *    speaker DAC and ES7210 dual-microphone ADC. Muse uses the BSP audio
 *    routines to share its I2S bus with the two independent codecs.
 */
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"

#include "muse_audio.h"
#include "muse_board.h"
#include "muse_pmu.h"

static const char *TAG = "board";

#define BOOT_GPIO GPIO_NUM_0
#define PMU_KEY_EVERY 2         /* poll latched PWR edges every 20 ms */

/* FT3168 touch controller: ensure its power mode is active at init. */
#define TOUCH_ADDR 0x38
#define TOUCH_REG_POWER_MODE 0xA5

static muse_gpio_button_t s_boot;

static esp_err_t touch_active_mode(void)
{
    /* The BSP's FT5x06 driver leaves the FT3168 in whatever power mode it
     * woke in; power-mode register 0xA5 = 0 selects active mode. */
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) {
        return ESP_ERR_INVALID_STATE;
    }
    i2c_master_dev_handle_t dev;
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TOUCH_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &dev), TAG, "touch dev");
    const uint8_t wr[2] = { TOUCH_REG_POWER_MODE, 0x00 };
    esp_err_t err = i2c_master_transmit(dev, wr, sizeof(wr), 50);
    i2c_master_bus_rm_device(dev);
    return err;
}

static esp_err_t init(void)
{
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c");
    ESP_RETURN_ON_ERROR(muse_gpio_button_init(&s_boot, BOOT_GPIO), TAG, "boot button");
    if (muse_pmu_init(bsp_i2c_get_handle(), true) != ESP_OK) {
        ESP_LOGW(TAG, "no PMU: PWR button and battery unavailable");
    } else {
        /* Discard the latched PWR press that just booted the device. */
        (void)muse_pmu_poll_key();
    }
    return ESP_OK;
}

static lv_display_t *display_start(lv_indev_t **touch)
{
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_stack = 8192;
    port_cfg.task_priority = 1;
    /* BSP v2.0.0 takes its buffer height from Kconfig, ignoring the buffer
     * fields here. The overlay sets CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT=8. */
    /*
     * The vendor bsp_display_start() sets buff_dma=true/buff_spiram=false.
     * Our custom config previously zero-initialized BOTH flags, losing that
     * safety invariant.  The SH8601 QSPI DMA engine must not receive generic
     * (potentially PSRAM-backed/non-DMA) LVGL draw buffers during PTT audio
     * and Wi-Fi traffic.  Keep a single small internal DMA buffer; the BSP
     * still derives its height from CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT=8.
     */
    const bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = port_cfg,
        .double_buffer = false,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
        },
    };
    lv_display_t *disp = bsp_display_start_with_config(&cfg);
    if (disp) {
        if (touch_active_mode() != ESP_OK) {
            ESP_LOGW(TAG, "FT3168 active-mode write failed; touch may sleep");
        }
    }
    *touch = disp ? bsp_display_get_input_dev() : NULL;
    return disp;
}

static bool display_lock(int timeout_ms)
{
    return bsp_display_lock(timeout_ms < 0 ? 0 : timeout_ms);
}

static void set_brightness(int pct)
{
    bsp_display_brightness_set(pct);
}

/*
 * The S3-2.06 has TWO audio ICs: ES8311 is the speaker DAC, while ES7210
 * digitizes the two onboard microphones. Use the vendor BSP's codec setup,
 * not an ES8311 ADC handle (which cannot receive ES7210's I2S data).
 */
static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    /* Muse opens a duplex 16 kHz, 16-bit, two-slot stream. The BSP default
     * (22.05 kHz mono) is not compatible, so initialize I2S explicitly. */
    const i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = BSP_I2S_MCLK,
            .bclk = BSP_I2S_SCLK,
            .ws = BSP_I2S_LCLK,
            .dout = BSP_I2S_DOUT,
            .din = BSP_I2S_DSIN,
        },
    };
    ESP_RETURN_ON_ERROR(bsp_audio_init(&std_cfg), TAG, "i2s");
    *spk = bsp_audio_codec_speaker_init();
    *mic = bsp_audio_codec_microphone_init();
    return *spk && *mic ? ESP_OK : ESP_FAIL;
}

static unsigned poll_buttons(void)
{
    static unsigned tick;
    unsigned ev = muse_gpio_button_poll(&s_boot);
    if (tick++ % PMU_KEY_EVERY == 0) {
        unsigned key = muse_pmu_poll_key();
        ev |= (key & MUSE_PMU_KEY_PRESS ? MUSE_BTN_AUX_PRESS : 0) |
              (key & MUSE_PMU_KEY_RELEASE ? MUSE_BTN_AUX_RELEASE : 0);
    }
    return ev;
}

static const muse_board_t s_board = {
    .name = "Waveshare ESP32-S3-Touch-AMOLED-2.06",
    .width = BSP_LCD_H_RES,
    .height = BSP_LCD_V_RES,
    .round = false,
    .touch = true,
    .diagonal_in = 2.06f,
    .talk_button = "boot",
    .aux_button = "pwr",
    .talk_hint = { LV_ALIGN_RIGHT_MID, -10, -140 },
    .aux_hint = { LV_ALIGN_RIGHT_MID, -10, 140 },
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = bsp_display_unlock,
    .set_brightness = set_brightness,
    .audio_init = audio_init,
    .mic_slot = -1,             /* ES7210 MIC1 + MIC2: mix the two microphones */
    .poll_buttons = poll_buttons,
    .read_power = muse_pmu_read_power,
    .power_off = muse_pmu_power_off,
};

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
