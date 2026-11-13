#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <driver/i2c_master.h>
#include <driver/i2s_std.h>
#include <driver/ledc.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <cmath>
#include <vector>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "CompactWifiBoard"

static int current_servo_angle = 90;
static int64_t last_servo_move_us = 0;

static void InitServo() {
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.duty_resolution = LEDC_TIMER_14_BIT;
    timer_conf.timer_num = LEDC_TIMER_1;
    timer_conf.freq_hz = 50;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num = SERVO_GPIO;
    ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf.channel = LEDC_CHANNEL_1;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.timer_sel = LEDC_TIMER_1;
    ch_conf.duty = 1228;
    ch_conf.hpoint = 0;
    ledc_channel_config(&ch_conf);
}

static void SetServoAngle(int angle) {
    if (angle < 30) angle = 30;
    if (angle > 150) angle = 150;
    if (abs(angle - current_servo_angle) < 15) return;

    int64_t now = esp_timer_get_time();
    if (now - last_servo_move_us < 450000) return;

    current_servo_angle = angle;
    last_servo_move_us = now;

    uint32_t duty = 410 + (angle * 1638 / 180);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

class StereoDoaAudioCodec : public AudioCodec {
private:
    i2s_chan_handle_t tx_handle_ = nullptr;
    i2s_chan_handle_t rx_handle_ = nullptr;

    void CreateSimplexChannels(gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout,
                               gpio_num_t mic_sck, gpio_num_t mic_ws, gpio_num_t mic_din) {
        i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
        tx_chan_cfg.dma_desc_num = 6;
        tx_chan_cfg.dma_frame_num = 240;
        tx_chan_cfg.auto_clear_after_cb = true;
        ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_handle_, nullptr));

        i2s_std_config_t tx_std_cfg = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)output_sample_rate_),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = spk_bclk,
                .ws = spk_ws,
                .dout = spk_dout,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
            },
        };
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &tx_std_cfg));

        i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
        rx_chan_cfg.dma_desc_num = 6;
        rx_chan_cfg.dma_frame_num = 240;
        ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, nullptr, &rx_handle_));

        i2s_std_config_t rx_std_cfg = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)input_sample_rate_),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = mic_sck,
                .ws = mic_ws,
                .dout = I2S_GPIO_UNUSED,
                .din = mic_din,
                .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
            },
        };
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &rx_std_cfg));
    }

        int Write(const int16_t* data, int samples) override {
        if (tx_handle_ == nullptr || !output_enabled_) return 0;
        std::vector<int16_t> buffer(samples);
        float volume_factor = pow(double(output_volume_) / 100.0, 2) * 65536.0;
        for (int i = 0; i < samples; i++) {
            int64_t temp = int64_t(data[i]) * int64_t(volume_factor);
            if (temp > 32767 * 65536) buffer[i] = 32767;
            else if (temp < -32768 * 65536) buffer[i] = -32768;
            else buffer[i] = temp >> 16;
        }
        size_t bytes_written = 0;
        i2s_channel_write(tx_handle_, buffer.data(), samples * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        return bytes_written / sizeof(int16_t);
    }

    int Read(int16_t* dest, int samples) override {
        if (rx_handle_ == nullptr || !input_enabled_) return 0;

        std::vector<int32_t> raw_stereo(samples * 2);
        size_t bytes_read = 0;
        if (i2s_channel_read(rx_handle_, raw_stereo.data(), raw_stereo.size() * sizeof(int32_t), &bytes_read, portMAX_DELAY) != ESP_OK) {
            return 0;
        }

        int frames = bytes_read / (2 * sizeof(int32_t));
        int64_t energy_left = 0;
        int64_t energy_right = 0;

        for (int i = 0; i < frames; i++) {
            int32_t left_val = raw_stereo[i * 2] >> 12;
            int32_t right_val = raw_stereo[i * 2 + 1] >> 12;

            if (left_val > 32767) left_val = 32767;
            else if (left_val < -32768) left_val = -32768;

            if (right_val > 32767) right_val = 32767;
            else if (right_val < -32768) right_val = -32768;

            energy_left += abs(left_val);
            energy_right += abs(right_val);

            int32_t mono_mix = (left_val + right_val) / 2;
            dest[i] = (int16_t)mono_mix;
        }

        int64_t since_move = esp_timer_get_time() - last_servo_move_us;
        if (since_move < 350000) {
            for (int i = 0; i < frames; i++) dest[i] = 0;
            return frames;
        }

        if (frames > 0 && (energy_left + energy_right) > (int64_t)frames * 1200) {
            if (energy_left > energy_right * 1.25) {
                SetServoAngle(45);
            } else if (energy_right > energy_left * 1.25) {
                SetServoAngle(135);
            } else {
                SetServoAngle(90);
            }
        }

        return frames;
    }

public:
    StereoDoaAudioCodec(int input_sample_rate, int output_sample_rate,
                        gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout,
                        gpio_num_t mic_sck, gpio_num_t mic_ws, gpio_num_t mic_din) {
        duplex_ = false;
        input_reference_ = false;
        input_channels_ = 1;
        input_sample_rate_ = input_sample_rate;
        output_sample_rate_ = output_sample_rate;
        CreateSimplexChannels(spk_bclk, spk_ws, spk_dout, mic_sck, mic_ws, mic_din);
    }

    ~StereoDoaAudioCodec() {
        if (rx_handle_ != nullptr) i2s_del_channel(rx_handle_);
        if (tx_handle_ != nullptr) i2s_del_channel(tx_handle_);
    }

    void Start() override {
        Settings settings("audio", false);
        output_volume_ = settings.GetInt("output_volume", output_volume_);
        if (output_volume_ <= 0) output_volume_ = 70;
        EnableInput(true);
        EnableOutput(true);
    }

    void EnableInput(bool enable) override {
        if (enable == input_enabled_) return;
        if (enable) i2s_channel_enable(rx_handle_);
        else i2s_channel_disable(rx_handle_);
        AudioCodec::EnableInput(enable);
    }

    void EnableOutput(bool enable) override {
        if (enable == output_enabled_) return;
        if (enable) i2s_channel_enable(tx_handle_);
        else i2s_channel_disable(tx_handle_);
        AudioCodec::EnableOutput(enable);
    }
};

class CompactWifiBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeSsd1306Display() {
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .scl_speed_hz = 400 * 1000,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
        };

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(display_i2c_bus_, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install SSD1306 driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

#ifdef SH1106
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
#endif
        ESP_LOGI(TAG, "SSD1306 driver installed");

        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize display");
            display_ = new NoDisplay();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));

        ESP_LOGI(TAG, "Turning display on");
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        touch_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        touch_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    void InitializeTools() {
        static LampController lamp(LAMP_GPIO);
    }

public:
    CompactWifiBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        touch_button_(TOUCH_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitServo();
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
        InitializeTools();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static StereoDoaAudioCodec audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(CompactWifiBoard);
