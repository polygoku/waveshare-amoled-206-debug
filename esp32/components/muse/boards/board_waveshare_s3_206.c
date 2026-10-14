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
 * ES8311 codec (speaker + mic), QMI8658 IMU, PCF85063 RTC — all on I2C0.
 * BOOT is GPIO0. Native USB Serial/JTAG (303a:1001).
 *
 * Pin/fact sources (per devices/AGENTS.md, do not guess pins):
 *  - Vendor wiki pinout: AMOLED QSPI SDIO0..3 = GPIO 4/5/6/7, SCLK = 11,
 *    CS = 12, RST = 8 (needs a LONG pulse: 200 ms low — short pulses don't
 *    boot the panel); I2C0 SDA = 15, SCL = 14; touch FT3168 RST = 9,
 *    INT = 38 (needs reg 0xA5 = monitor mode at init); audio ES8311 I2S0
 *    DOUT = 40, BCLK = 41, LRC = 45, MCLK = 16, PA enable = 46 (high only
 *    while playing audio); mic ASDOUT = 42.
 *  - Managed BSP: waveshare/esp32_s3_touch_amoled_2_06 ^2.0.0 (ESP Component
 *    Registry), which drives the CO5300 panel, FT3168 touch, ES8311 and
 *    AXP2101. Muse uses it for display, touch, I2C and PMU; audio is wired
 *    by hand so the ES8311 runs in ADC+DAC mode for the onboard mic.
 */
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"

#include "muse_audio.h"
#include "muse_board.h"
#include "muse_pmu.h"

static const char *TAG = "board";

#define BOOT_GPIO GPIO_NUM_0
#define PMU_KEY_EVERY 2         /* poll latched PWR edges every 20 ms */

/* FT3168 touch controller: put it in monitor (active) mode at init. */
#define TOUCH_ADDR 0x38
#define TOUCH_REG_POWER_MODE 0xA5

static muse_gpio_button_t s_boot;

static esp_err_t touch_monitor_mode(void)
{
    /* The BSP's FT5x06 driver leaves the FT3168 in whatever power mode it
     * woke in; register 0xA5 = 0 selects monitor (active) mode. */
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
    const bsp_display_cfg_t cfg = { .lvgl_port_cfg = port_cfg };
    lv_display_t *disp = bsp_display_start_with_config(&cfg);
    if (disp) {
        if (touch_monitor_mode() != ESP_OK) {
            ESP_LOGW(TAG, "FT3168 monitor-mode write failed; touch may sleep");
        }
    }
    *touch = bsp_display_get_input_dev();
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
 * ES8311 in ADC+DAC mode: speaker on I2S0 TX, onboard mic on I2S0 RX
 * (ASDOUT = GPIO42, slaved to the codec clocks). PA enable (GPIO46) is
 * driven high by the codec driver only while the speaker is open.
 */
static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    i2s_chan_handle_t tx, rx;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx, &rx), TAG, "i2s channel");
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
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx, &std_cfg), TAG, "i2s tx");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx, &std_cfg), TAG, "i2s rx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx), TAG, "i2s tx on");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx), TAG, "i2s rx on");

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = CONFIG_BSP_I2S_NUM,
        .rx_handle = rx,
        .tx_handle = tx,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = bsp_i2c_get_handle(),
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(data_if && ctrl_if && gpio_if, ESP_ERR_NO_MEM, TAG, "codec interfaces");

    es8311_codec_cfg_t es_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,   /* PA enable is active high */
        .master_mode = false,
        .use_mclk = true,
        .hw_gain = { .pa_voltage = 5.0, .codec_dac_voltage = 3.3 },
    };
    const audio_codec_if_t *codec = es8311_codec_new(&es_cfg);
    ESP_RETURN_ON_FALSE(codec, ESP_FAIL, TAG, "ES8311 not responding");

    esp_codec_dev_cfg_t out_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = codec,
        .data_if = data_if,
    };
    esp_codec_dev_cfg_t in_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = codec,
        .data_if = data_if,
    };
    *spk = esp_codec_dev_new(&out_cfg);
    *mic = esp_codec_dev_new(&in_cfg);
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
    .mic_slot = 0,              /* one mic on the ES8311 ADC */
    .poll_buttons = poll_buttons,
    .read_power = muse_pmu_read_power,
    .power_off = muse_pmu_power_off,
};

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
