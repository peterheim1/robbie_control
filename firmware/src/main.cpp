/*
 * ReSpeaker Lite TCP Audio Firmware
 *
 * Turns the ReSpeaker Lite (XIAO ESP32-S3) into a network microphone/speaker.
 * Streams I2S mic audio to a TCP server and plays back TTS audio received
 * over TCP.
 *
 * TCP protocol: 5-byte header [msg_type(1) | length(4 big-endian)] + payload
 *   0x01 AUDIO_MIC  ESP32→PC  raw 16-bit PCM @ 16kHz
 *   0x02 AUDIO_TTS  PC→ESP32  raw 16-bit PCM @ 16kHz
 *   0x03 TTS_START  PC→ESP32  signals start of TTS playback
 *   0x04 TTS_END    PC→ESP32  signals end of TTS playback
 *   0x05 LED_STATE  PC→ESP32  1 byte: 0=idle(blue), 1=wake(magenta), 2=thinking(yellow)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <driver/i2s_std.h>
#include <FastLED.h>

// ── Network ──────────────────────────────────────────────────────────────────
static const char* WIFI_SSID     = "TelstraA6F96D";
static const char* WIFI_PASS     = "05E3469DCD";

static const IPAddress STATIC_IP(10, 0, 0, 25);
static const IPAddress GATEWAY(10, 0, 0, 138);
static const IPAddress SUBNET(255, 255, 255, 0);
static const IPAddress DNS(10, 0, 0, 138);

static const char*    SERVER_HOST = "10.0.0.87";
static const uint16_t SERVER_PORT = 8765;

// ── I2S pins (ReSpeaker Lite XIAO ESP32-S3) ─────────────────────────────────
static const int PIN_BCLK  = 8;
static const int PIN_LRCLK = 7;
static const int PIN_MCLK  = 9;    // master clock from XU316
static const int PIN_DIN   = 44;   // mic data (I2S RX)
static const int PIN_DOUT  = 43;   // speaker data (I2S TX)

// ── Audio ────────────────────────────────────────────────────────────────────
// XU316 runs I2S at 48kHz stereo 32-bit; we downsample to 16kHz mono 16-bit for TCP
static const int      I2S_SAMPLE_RATE   = 48000;
static const int      OUT_SAMPLE_RATE   = 16000;  // what we send over TCP
static const int      MIC_FRAME_SAMPLES = 512;    // mono output samples per read
// I2S reads stereo 32-bit: 2 channels × 4 bytes per sample
static const int      I2S_FRAME_BYTES   = MIC_FRAME_SAMPLES * 3 * 2 * sizeof(int32_t);
// ^ 3x oversampled (48k/16k=3) × stereo × 4 bytes
// Mono 16-bit output sent over TCP
static const int      MIC_FRAME_BYTES   = MIC_FRAME_SAMPLES * sizeof(int16_t);

// TTS buffer in PSRAM (max ~10 seconds of audio)
static const size_t   TTS_BUF_MAX = OUT_SAMPLE_RATE * sizeof(int16_t) * 10;

// ── LED ──────────────────────────────────────────────────────────────────────
static const int PIN_LED   = 1;
static const int NUM_LEDS  = 1;
static CRGB leds[NUM_LEDS];

// ── Protocol message types ───────────────────────────────────────────────────
static const uint8_t MSG_AUDIO_MIC = 0x01;
static const uint8_t MSG_AUDIO_TTS = 0x02;
static const uint8_t MSG_TTS_START = 0x03;
static const uint8_t MSG_TTS_END   = 0x04;
static const uint8_t MSG_LED_STATE = 0x05;

// LED state values (from server)
static const uint8_t LED_IDLE     = 0;
static const uint8_t LED_WAKE     = 1;
static const uint8_t LED_THINKING = 2;

// ── I2S channel handles (new driver API) ─────────────────────────────────────
static i2s_chan_handle_t rx_chan = nullptr;  // mic
static i2s_chan_handle_t tx_chan = nullptr;  // speaker

// ── Shared state ─────────────────────────────────────────────────────────────
static WiFiClient       tcpClient;
static SemaphoreHandle_t tcpMutex;
static volatile bool     connected    = false;
static volatile bool     ttsPlaying   = false;
static volatile bool     chimeRequested = false;

// TTS buffer (allocated in PSRAM)
static uint8_t* ttsBuf     = nullptr;
static size_t   ttsBufLen  = 0;
static SemaphoreHandle_t ttsMutex;
static TaskHandle_t      ttsTaskHandle = nullptr;

// ── Helpers ──────────────────────────────────────────────────────────────────

static void setLED(CRGB color) {
    leds[0] = color;
    FastLED.show();
}

static bool sendHeader(uint8_t msgType, uint32_t length) {
    uint8_t hdr[5];
    hdr[0] = msgType;
    hdr[1] = (length >> 24) & 0xFF;
    hdr[2] = (length >> 16) & 0xFF;
    hdr[3] = (length >>  8) & 0xFF;
    hdr[4] =  length        & 0xFF;
    return tcpClient.write(hdr, 5) == 5;
}

static bool readExact(uint8_t* buf, size_t len, unsigned long timeoutMs = 5000) {
    size_t received = 0;
    unsigned long start = millis();
    while (received < len) {
        if (!tcpClient.connected()) return false;
        if (millis() - start > timeoutMs) return false;
        int avail = tcpClient.available();
        if (avail > 0) {
            int toRead = min((size_t)avail, len - received);
            int got = tcpClient.read(buf + received, toRead);
            if (got > 0) {
                received += got;
                start = millis();  // reset timeout on progress
            }
        } else {
            vTaskDelay(1);
        }
    }
    return true;
}

// ── I2S setup (new driver API) ───────────────────────────────────────────────

static void i2sInit() {
    // Allocate a full-duplex channel pair (TX + RX share the same I2S port)
    // XU316 is I2S master; ESP32 must be slave
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    chan_cfg.dma_desc_num = 8;
    chan_cfg.dma_frame_num = MIC_FRAME_SAMPLES * 3;  // 3x for 48k→16k downsample
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan));

    // XU316 runs at 48kHz stereo 32-bit (confirmed by ESPHome integration config)
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = (gpio_num_t)PIN_MCLK,
            .bclk = (gpio_num_t)PIN_BCLK,
            .ws   = (gpio_num_t)PIN_LRCLK,
            .dout = (gpio_num_t)PIN_DOUT,
            .din  = (gpio_num_t)PIN_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));

    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
}

// ── micTask: read I2S DMA → TCP ─────────────────────────────────────────────

static void micTask(void* param) {
    // Read buffer: stereo 32-bit from I2S (48kHz)
    int32_t* i2sBuf = (int32_t*)malloc(I2S_FRAME_BYTES);
    // Output buffer: mono 16-bit for TCP (16kHz)
    int16_t* outBuf = (int16_t*)malloc(MIC_FRAME_BYTES);
    if (!i2sBuf || !outBuf) {
        Serial.println("[mic] malloc failed");
        vTaskDelete(nullptr);
        return;
    }

    while (true) {
        if (ttsPlaying) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        size_t bytesRead = 0;
        esp_err_t err = i2s_channel_read(rx_chan, i2sBuf, I2S_FRAME_BYTES,
                                          &bytesRead, portMAX_DELAY);
        if (err == ESP_OK && bytesRead > 0 && connected) {
            // Convert 48kHz stereo 32-bit → 16kHz mono 16-bit
            // Stereo pairs: [L0,R0, L1,R1, L2,R2, ...] as int32
            size_t stereoSamples = bytesRead / sizeof(int32_t);
            size_t monoSamples48k = stereoSamples / 2;
            // Downsample 48k→16k: take every 3rd sample
            size_t monoSamples16k = monoSamples48k / 3;
            if (monoSamples16k > (size_t)MIC_FRAME_SAMPLES)
                monoSamples16k = MIC_FRAME_SAMPLES;

            // Diagnostic: log audio stats every ~1 second
            static int diagCount = 0;
            if (++diagCount >= 32) {
                diagCount = 0;
                int32_t lMin = 0x7FFFFFFF, lMax = -0x7FFFFFFF;
                for (size_t i = 0; i < monoSamples48k && i < 512; i++) {
                    int32_t l = i2sBuf[i * 2];
                    if (l < lMin) lMin = l;
                    if (l > lMax) lMax = l;
                }
                Serial.printf("[mic] L(32): %ld..%ld  out16k=%u samples\n",
                              (long)lMin, (long)lMax, monoSamples16k);
            }

            for (size_t i = 0; i < monoSamples16k; i++) {
                // Take left channel, every 3rd sample; shift 32→16 bit
                int32_t sample = i2sBuf[i * 3 * 2] >> 12;
                if (sample > 32767) sample = 32767;
                if (sample < -32768) sample = -32768;
                outBuf[i] = (int16_t)sample;
            }
            size_t outBytes = monoSamples16k * sizeof(int16_t);

            if (xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(10))) {
                if (tcpClient.connected()) {
                    sendHeader(MSG_AUDIO_MIC, outBytes);
                    tcpClient.write((uint8_t*)outBuf, outBytes);
                }
                xSemaphoreGive(tcpMutex);
            }
        }
    }
}

// ── ttsTask: play buffered TTS audio via I2S ────────────────────────────────
// Input: 16kHz mono 16-bit PCM in ttsBuf
// Output: 48kHz stereo 32-bit to I2S (matching XU316 codec format)
// Conversion: 3x linear interpolation, <<12 to 32-bit, mono→stereo

// Process 256 input samples at a time → 768 output frames × 2 ch × 4 bytes = 6144 bytes
static const size_t TTS_IN_CHUNK   = 256;
static const size_t TTS_OUT_FRAMES = TTS_IN_CHUNK * 3;  // 768 after 3x upsample
static const size_t TTS_OUT_BYTES  = TTS_OUT_FRAMES * 2 * sizeof(int32_t);  // stereo 32-bit

static void playChime(int32_t* outBuf) {
    // Generate 200ms 880Hz sine wave with fade-in/fade-out, output as 48kHz/32-bit/stereo
    const int durationMs = 200;
    const int totalFrames = (I2S_SAMPLE_RATE * durationMs) / 1000;  // 9600
    const float freq = 880.0f;
    const int fadeFrames = totalFrames / 5;  // 40ms fade

    int remaining = totalFrames;
    int frameIdx = 0;
    // Reuse outBuf in chunks
    while (remaining > 0) {
        int frames = min((int)TTS_OUT_FRAMES, remaining);
        for (int i = 0; i < frames; i++) {
            float t = (float)(frameIdx + i) / (float)I2S_SAMPLE_RATE;
            float sample = sinf(2.0f * M_PI * freq * t);

            // Apply envelope (fade in/out)
            int pos = frameIdx + i;
            float env = 1.0f;
            if (pos < fadeFrames)
                env = (float)pos / (float)fadeFrames;
            else if (pos >= totalFrames - fadeFrames)
                env = (float)(totalFrames - 1 - pos) / (float)fadeFrames;

            int32_t val = (int32_t)(sample * env * 2000.0f) << 12;
            outBuf[i * 2]     = val;  // left
            outBuf[i * 2 + 1] = val;  // right
        }
        size_t written = 0;
        i2s_channel_write(tx_chan, outBuf, frames * 2 * sizeof(int32_t),
                          &written, portMAX_DELAY);
        frameIdx += frames;
        remaining -= frames;
    }
    Serial.println("[tts] Chime played");
}

static void ttsTask(void* param) {
    // Allocate conversion buffer in PSRAM: stereo 32-bit output
    int32_t* outChunk = (int32_t*)ps_malloc(TTS_OUT_BYTES);
    if (!outChunk) {
        Serial.println("[tts] PSRAM alloc for conversion buffer failed");
        vTaskDelete(nullptr);
        return;
    }

    while (true) {
        // Poll with 50ms timeout to check for chime requests
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));

        // Check chime request (set by MSG_LED_STATE wake)
        if (chimeRequested) {
            chimeRequested = false;
            ttsPlaying = true;
            playChime(outChunk);
            ttsPlaying = false;
        }

        if (!notified) continue;  // no TTS data, loop back to poll

        if (xSemaphoreTake(ttsMutex, portMAX_DELAY)) {
            size_t len = ttsBufLen;
            xSemaphoreGive(ttsMutex);

            if (len == 0) continue;

            ttsPlaying = true;
            size_t totalSamples = len / sizeof(int16_t);
            Serial.printf("[tts] Playing %u bytes (%u samples), converting 16k→48k\n",
                          len, totalSamples);

            int16_t* inBuf = (int16_t*)ttsBuf;
            size_t offset = 0;  // input sample index
            int16_t prevSample = 0;

            while (offset < totalSamples) {
                size_t chunk = min(TTS_IN_CHUNK, totalSamples - offset);
                size_t outIdx = 0;

                for (size_t i = 0; i < chunk; i++) {
                    int16_t cur = inBuf[offset + i];
                    // 3x linear interpolation between previous and current sample
                    // Produces 3 output frames per input sample
                    int32_t s0 = (int32_t)prevSample;
                    int32_t s1 = (int32_t)cur;

                    int32_t interp0 = (s0 * 2 + s1 * 1) / 3;
                    int32_t interp1 = (s0 * 1 + s1 * 2) / 3;

                    // Frame 0: interpolated 1/3
                    int32_t val0 = interp0 << 12;
                    outChunk[outIdx++] = val0;  // L
                    outChunk[outIdx++] = val0;  // R

                    // Frame 1: interpolated 2/3
                    int32_t val1 = interp1 << 12;
                    outChunk[outIdx++] = val1;  // L
                    outChunk[outIdx++] = val1;  // R

                    // Frame 2: current sample
                    int32_t val2 = s1 << 12;
                    outChunk[outIdx++] = val2;  // L
                    outChunk[outIdx++] = val2;  // R

                    prevSample = cur;
                }

                size_t outBytes = outIdx * sizeof(int32_t);
                size_t written = 0;
                i2s_channel_write(tx_chan, outChunk, outBytes,
                                  &written, portMAX_DELAY);
                offset += chunk;
            }

            // Flush with silence (stereo 32-bit)
            memset(outChunk, 0, TTS_OUT_BYTES);
            for (int i = 0; i < 4; i++) {
                size_t w = 0;
                i2s_channel_write(tx_chan, outChunk, TTS_OUT_BYTES, &w, portMAX_DELAY);
            }

            ttsPlaying = false;

            if (xSemaphoreTake(ttsMutex, portMAX_DELAY)) {
                ttsBufLen = 0;
                xSemaphoreGive(ttsMutex);
            }

            Serial.println("[tts] Playback done");
        }
    }
}

// ── netTask: TCP connection + receive TTS ───────────────────────────────────

static void netTask(void* param) {
    while (true) {
        // Connect
        while (!tcpClient.connected()) {
            connected = false;
            setLED(CRGB::Black);
            Serial.printf("[net] Connecting to %s:%d...\n", SERVER_HOST, SERVER_PORT);
            if (tcpClient.connect(SERVER_HOST, SERVER_PORT)) {
                connected = true;
                setLED(CRGB::Blue);
                Serial.println("[net] Connected");
                tcpClient.setNoDelay(true);
            } else {
                Serial.println("[net] Connection failed, retrying in 2s");
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }

        // Receive loop: read headers and dispatch
        while (tcpClient.connected()) {
            if (tcpClient.available() < 5) {
                vTaskDelay(1);
                continue;
            }

            uint8_t hdr[5];
            if (!readExact(hdr, 5)) {
                Serial.println("[net] Header read failed");
                break;
            }

            uint8_t msgType = hdr[0];
            uint32_t length = ((uint32_t)hdr[1] << 24) |
                              ((uint32_t)hdr[2] << 16) |
                              ((uint32_t)hdr[3] <<  8) |
                               (uint32_t)hdr[4];

            switch (msgType) {
                case MSG_AUDIO_TTS: {
                    if (length > TTS_BUF_MAX) {
                        Serial.printf("[net] TTS too large: %u\n", length);
                        uint8_t drain[256];
                        size_t remaining = length;
                        while (remaining > 0) {
                            size_t chunk = min(remaining, sizeof(drain));
                            if (!readExact(drain, chunk)) break;
                            remaining -= chunk;
                        }
                        break;
                    }
                    if (xSemaphoreTake(ttsMutex, portMAX_DELAY)) {
                        // Append to TTS buffer (multiple chunks arrive between START/END)
                        if (ttsBufLen + length <= TTS_BUF_MAX) {
                            if (!readExact(ttsBuf + ttsBufLen, length)) {
                                xSemaphoreGive(ttsMutex);
                                Serial.println("[net] TTS read failed");
                                break;
                            }
                            ttsBufLen += length;
                        } else {
                            // Buffer full, drain and discard
                            xSemaphoreGive(ttsMutex);
                            uint8_t drain[256];
                            size_t remaining = length;
                            while (remaining > 0) {
                                size_t chunk = min(remaining, sizeof(drain));
                                if (!readExact(drain, chunk)) break;
                                remaining -= chunk;
                            }
                            break;
                        }
                        xSemaphoreGive(ttsMutex);
                    }
                    break;
                }
                case MSG_TTS_START:
                    Serial.println("[net] TTS_START");
                    if (xSemaphoreTake(ttsMutex, portMAX_DELAY)) {
                        ttsBufLen = 0;
                        xSemaphoreGive(ttsMutex);
                    }
                    break;

                case MSG_TTS_END:
                    Serial.println("[net] TTS_END → play");
                    xTaskNotifyGive(ttsTaskHandle);
                    break;

                case MSG_LED_STATE: {
                    if (length >= 1) {
                        uint8_t state;
                        if (!readExact(&state, 1)) break;
                        // Drain any extra bytes
                        if (length > 1) {
                            uint8_t drain[256];
                            size_t rem = length - 1;
                            while (rem > 0) {
                                size_t chunk = min(rem, sizeof(drain));
                                if (!readExact(drain, chunk)) break;
                                rem -= chunk;
                            }
                        }
                        switch (state) {
                            case LED_IDLE:
                                setLED(CRGB::Blue);
                                Serial.println("[net] LED → idle (blue)");
                                break;
                            case LED_WAKE:
                                setLED(CRGB::Magenta);
                                Serial.println("[net] LED → wake (magenta)");
                                chimeRequested = true;
                                xTaskNotifyGive(ttsTaskHandle);  // wake ttsTask from poll
                                break;
                            case LED_THINKING:
                                setLED(CRGB::Yellow);
                                Serial.println("[net] LED → thinking (yellow)");
                                break;
                            default:
                                Serial.printf("[net] Unknown LED state: %u\n", state);
                                break;
                        }
                    }
                    break;
                }

                default:
                    Serial.printf("[net] Unknown msg type: 0x%02X\n", msgType);
                    if (length > 0) {
                        uint8_t drain[256];
                        size_t remaining = length;
                        while (remaining > 0) {
                            size_t chunk = min(remaining, sizeof(drain));
                            if (!readExact(drain, chunk)) break;
                            remaining -= chunk;
                        }
                    }
                    break;
            }
        }

        // Disconnected
        connected = false;
        setLED(CRGB::Black);
        tcpClient.stop();
        Serial.println("[net] Disconnected");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ── Arduino setup / loop ─────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== ReSpeaker Lite TCP Audio ===");

    // LED
    FastLED.addLeds<WS2812, PIN_LED, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(30);
    setLED(CRGB::Red);

    // WiFi
    WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("[wifi] Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\n[wifi] Connected: %s\n", WiFi.localIP().toString().c_str());
    setLED(CRGB::Yellow);

    // OTA
    ArduinoOTA.setHostname("respeaker-lite");
    ArduinoOTA.onStart([]() {
        Serial.println("[ota] Update starting...");
        setLED(CRGB::White);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("[ota] Update complete!");
        setLED(CRGB::Green);
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[ota] %u%%\r", progress * 100 / total);
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[ota] Error %u\n", error);
        setLED(CRGB::Red);
    });
    ArduinoOTA.begin();
    Serial.println("[ota] ArduinoOTA ready");

    // I2S
    i2sInit();
    Serial.println("[i2s] Initialized");

    // Allocate TTS buffer in PSRAM
    ttsBuf = (uint8_t*)ps_malloc(TTS_BUF_MAX);
    if (!ttsBuf) {
        Serial.println("[ERROR] PSRAM alloc failed!");
        while (true) delay(1000);
    }
    Serial.printf("[mem] TTS buffer: %u bytes in PSRAM\n", TTS_BUF_MAX);

    // Semaphores
    tcpMutex = xSemaphoreCreateMutex();
    ttsMutex = xSemaphoreCreateMutex();

    // Tasks
    xTaskCreatePinnedToCore(micTask, "mic", 4096, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(ttsTask, "tts", 8192, nullptr, 2, &ttsTaskHandle, 1);
    xTaskCreatePinnedToCore(netTask, "net", 8192, nullptr, 1, nullptr, 0);

    Serial.println("[setup] All tasks started");
}

void loop() {
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(50));
}
