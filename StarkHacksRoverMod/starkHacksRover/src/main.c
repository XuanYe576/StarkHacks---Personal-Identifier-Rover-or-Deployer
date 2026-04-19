// ESP32 Rover Controller (SoftAP + UDP)
// Packet format (CSV):
//   vx,vy,wz,elev,en           (recommended; single elevation servo)
//   vx,vy,wz,s2,s3,s4,en       (legacy accepted; s2 used as elevation)
//   vx,vy,wz,s1,s2,s3,s4,en    (legacy accepted; s1 ignored, s2 used)
// Example:
//   0.40,0.00,-0.20,95,1
// Also supports single-token keyboard commands over UDP:
//   W A S D Q E X
//   W/S: forward/backward, A/D: strafe, Q/E: rotate, X: stop
//
// vx: forward/backward   [-1..1]
// vy: strafe left/right  [-1..1]
// wz: yaw rotate         [-1..1]
// elev: elevation servo angle [0..180]
// en: 0/1 enable motors

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#define WIFI_AP_SSID "TzariumRover"
#define WIFI_AP_PASS "rover1234"
#define WIFI_AP_CHANNEL 6
#define WIFI_MAX_CONN 4

#define UDP_PORT 7001
#define RX_BUF_SIZE 256

#define CONTROL_TIMEOUT_MS 220
#define CONTROL_LOOP_MS 20
#define WASD_LINEAR_SPEED 0.60f
#define WASD_ROTATE_SPEED 0.45f

#define MOTOR_PWM_FREQ_HZ 20000
#define MOTOR_PWM_RES LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX ((1 << 10) - 1)

#define SERVO_PWM_FREQ_HZ 50
// adafruit_qtpy_esp32s3_n4r2 (ESP-IDF in your setup) supports up to 14-bit LEDC.
#define SERVO_PWM_RES LEDC_TIMER_14_BIT
#define SERVO_PWM_MAX ((1 << 14) - 1)

// Servo pulse width mapping (typical)
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

// -------------------------
// Pin map (edit for your board)
// -------------------------
// Motor order: FL, FR, RL, RR
static const int MOTOR_IN1[4] = {4, 6, 8, 10};
static const int MOTOR_IN2[4] = {5, 7, 9, 11};
static const int MOTOR_PWM_PIN[4] = {12, 13, 14, 15};

// Single optional elevation servo. Set to -1 to fully disable servo output.
static const int ELEVATION_SERVO_PIN = 17;

// TB6612 STBY pins (optional). Set -1 if STBY is tied high in hardware.
static const int TB6612_FRONT_STBY_PIN = -1;
static const int TB6612_REAR_STBY_PIN = -1;

static const ledc_channel_t MOTOR_PWM_CH[4] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
static const ledc_channel_t ELEVATION_SERVO_PWM_CH = LEDC_CHANNEL_4;

typedef struct {
    float vx;
    float vy;
    float wz;
    int elevation_deg;
    int enable;
    int64_t last_rx_us;
} control_state_t;

static const char *TAG = "rovermod";
static SemaphoreHandle_t g_ctrl_mutex;
static control_state_t g_ctrl = {
    .vx = 0.0f,
    .vy = 0.0f,
    .wz = 0.0f,
    .elevation_deg = 90,
    .enable = 0,
    .last_rx_us = 0,
};

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline int clampi(int v, int lo, int hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static uint32_t servo_deg_to_duty(int deg) {
    deg = clampi(deg, 0, 180);
    uint32_t pulse_us = SERVO_MIN_US + (uint32_t)((SERVO_MAX_US - SERVO_MIN_US) * (deg / 180.0f));
    uint32_t period_us = 1000000UL / SERVO_PWM_FREQ_HZ;
    uint32_t duty = (uint32_t)((pulse_us * (uint64_t)SERVO_PWM_MAX) / period_us);
    if (duty > SERVO_PWM_MAX) duty = SERVO_PWM_MAX;
    return duty;
}

static void set_motor(int idx, float cmd) {
    cmd = clampf(cmd, -1.0f, 1.0f);
    int in1 = MOTOR_IN1[idx];
    int in2 = MOTOR_IN2[idx];
    int pwm_pin = MOTOR_PWM_PIN[idx];
    if (in1 < 0 || in2 < 0 || pwm_pin < 0) return;

    int duty = (int)(fabsf(cmd) * MOTOR_PWM_MAX);
    if (cmd > 0.01f) {
        gpio_set_level((gpio_num_t)in1, 1);
        gpio_set_level((gpio_num_t)in2, 0);
    } else if (cmd < -0.01f) {
        gpio_set_level((gpio_num_t)in1, 0);
        gpio_set_level((gpio_num_t)in2, 1);
    } else {
        gpio_set_level((gpio_num_t)in1, 0);
        gpio_set_level((gpio_num_t)in2, 0);
        duty = 0;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CH[idx], duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CH[idx]);
}

static void set_elevation_servo(int deg) {
    if (ELEVATION_SERVO_PIN < 0) return;
    uint32_t duty = servo_deg_to_duty(deg);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ELEVATION_SERVO_PWM_CH, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ELEVATION_SERVO_PWM_CH);
}

static void stop_all_motors(void) {
    for (int i = 0; i < 4; i++) {
        set_motor(i, 0.0f);
    }
}

static void apply_mecanum(const control_state_t *st) {
    if (!st->enable) {
        stop_all_motors();
        return;
    }

    float vx = clampf(st->vx, -1.0f, 1.0f);
    float vy = clampf(st->vy, -1.0f, 1.0f);
    float wz = clampf(st->wz, -1.0f, 1.0f);

    // FL, FR, RL, RR
    float m[4];
    m[0] = vx + vy + wz;
    m[1] = vx - vy - wz;
    m[2] = vx - vy + wz;
    m[3] = vx + vy - wz;

    float maxabs = 1.0f;
    for (int i = 0; i < 4; i++) {
        float a = fabsf(m[i]);
        if (a > maxabs) maxabs = a;
    }
    for (int i = 0; i < 4; i++) {
        m[i] /= maxabs;
        set_motor(i, m[i]);
    }
}

static void init_pwm_and_gpio(void) {
    ledc_timer_config_t motor_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&motor_timer);

    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_PWM_RES,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = SERVO_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&servo_timer);

    for (int i = 0; i < 4; i++) {
        if (MOTOR_IN1[i] >= 0) {
            gpio_reset_pin((gpio_num_t)MOTOR_IN1[i]);
            gpio_set_direction((gpio_num_t)MOTOR_IN1[i], GPIO_MODE_OUTPUT);
            gpio_set_level((gpio_num_t)MOTOR_IN1[i], 0);
        }
        if (MOTOR_IN2[i] >= 0) {
            gpio_reset_pin((gpio_num_t)MOTOR_IN2[i]);
            gpio_set_direction((gpio_num_t)MOTOR_IN2[i], GPIO_MODE_OUTPUT);
            gpio_set_level((gpio_num_t)MOTOR_IN2[i], 0);
        }
        if (MOTOR_PWM_PIN[i] >= 0) {
            ledc_channel_config_t ch = {
                .gpio_num = MOTOR_PWM_PIN[i],
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = MOTOR_PWM_CH[i],
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = LEDC_TIMER_0,
                .duty = 0,
                .hpoint = 0,
            };
            ledc_channel_config(&ch);
        }
    }

    if (ELEVATION_SERVO_PIN >= 0) {
        ledc_channel_config_t ch = {
            .gpio_num = ELEVATION_SERVO_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = ELEVATION_SERVO_PWM_CH,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_1,
            .duty = servo_deg_to_duty(90),
            .hpoint = 0,
        };
        ledc_channel_config(&ch);
    }

    if (TB6612_FRONT_STBY_PIN >= 0) {
        gpio_reset_pin((gpio_num_t)TB6612_FRONT_STBY_PIN);
        gpio_set_direction((gpio_num_t)TB6612_FRONT_STBY_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)TB6612_FRONT_STBY_PIN, 1);
    }
    if (TB6612_REAR_STBY_PIN >= 0) {
        gpio_reset_pin((gpio_num_t)TB6612_REAR_STBY_PIN);
        gpio_set_direction((gpio_num_t)TB6612_REAR_STBY_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)TB6612_REAR_STBY_PIN, 1);
    }
}

static void wifi_init_softap(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t ap_config = {0};
    strncpy((char *)ap_config.ap.ssid, WIFI_AP_SSID, sizeof(ap_config.ap.ssid) - 1);
    strncpy((char *)ap_config.ap.password, WIFI_AP_PASS, sizeof(ap_config.ap.password) - 1);
    ap_config.ap.ssid_len = strlen(WIFI_AP_SSID);
    ap_config.ap.channel = WIFI_AP_CHANNEL;
    ap_config.ap.max_connection = WIFI_MAX_CONN;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    if (strlen(WIFI_AP_PASS) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "SoftAP started SSID=%s PASS=%s CH=%d", WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL);
}

static int parse_csv_command(const char *line, control_state_t *out) {
    float vx, vy, wz;
    int elev, en;
    int matched = sscanf(line, " %f , %f , %f , %d , %d", &vx, &vy, &wz, &elev, &en);
    if (matched == 5) {
        out->vx = clampf(vx, -1.0f, 1.0f);
        out->vy = clampf(vy, -1.0f, 1.0f);
        out->wz = clampf(wz, -1.0f, 1.0f);
        out->elevation_deg = clampi(elev, 0, 180);
        out->enable = (en != 0) ? 1 : 0;
        out->last_rx_us = esp_timer_get_time();
        return 1;
    }

    int s1, s2, s3, s4;
    matched = sscanf(line, " %f , %f , %f , %d , %d , %d , %d , %d",
                     &vx, &vy, &wz, &s1, &s2, &s3, &s4, &en);
    if (matched == 8) {
        out->vx = clampf(vx, -1.0f, 1.0f);
        out->vy = clampf(vy, -1.0f, 1.0f);
        out->wz = clampf(wz, -1.0f, 1.0f);
        // Keep backward compatibility: use s2 as elevation.
        out->elevation_deg = clampi(s2, 0, 180);
        out->enable = (en != 0) ? 1 : 0;
        out->last_rx_us = esp_timer_get_time();
        return 1;
    }

    // Legacy compact format: vx,vy,wz,s2,s3,s4,en
    int n2, n3, n4, nen;
    matched = sscanf(line, " %f , %f , %f , %d , %d , %d , %d",
                     &vx, &vy, &wz, &n2, &n3, &n4, &nen);
    if (matched != 7) return 0;

    out->vx = clampf(vx, -1.0f, 1.0f);
    out->vy = clampf(vy, -1.0f, 1.0f);
    out->wz = clampf(wz, -1.0f, 1.0f);
    out->elevation_deg = clampi(n2, 0, 180);
    out->enable = (nen != 0) ? 1 : 0;
    out->last_rx_us = esp_timer_get_time();
    return 1;
}

static int parse_wasd_command(const char *line, control_state_t *out) {
    if (line == NULL || line[0] == '\0') return 0;
    char c = (char)toupper((unsigned char)line[0]);
    if (!(c == 'W' || c == 'A' || c == 'S' || c == 'D' || c == 'Q' || c == 'E' || c == 'X')) {
        return 0;
    }

    out->vx = 0.0f;
    out->vy = 0.0f;
    out->wz = 0.0f;
    out->enable = 1;

    if (c == 'W') out->vx = WASD_LINEAR_SPEED;
    if (c == 'S') out->vx = -WASD_LINEAR_SPEED;
    if (c == 'A') out->vy = WASD_LINEAR_SPEED;
    if (c == 'D') out->vy = -WASD_LINEAR_SPEED;
    if (c == 'Q') out->wz = WASD_ROTATE_SPEED;
    if (c == 'E') out->wz = -WASD_ROTATE_SPEED;
    if (c == 'X') out->enable = 0;

    out->last_rx_us = esp_timer_get_time();
    return 1;
}

static void udp_server_task(void *arg) {
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP server listening on port %d", UDP_PORT);
    char buf[RX_BUF_SIZE];
    while (1) {
        struct sockaddr_in src_addr;
        socklen_t socklen = sizeof(src_addr);
        int len = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&src_addr, &socklen);
        if (len <= 0) continue;

        buf[len] = '\0';
        for (int i = len - 1; i >= 0; i--) {
            if (buf[i] == '\r' || buf[i] == '\n' || isspace((unsigned char)buf[i])) {
                buf[i] = '\0';
            } else {
                break;
            }
        }

        control_state_t next = {0};
        if (xSemaphoreTake(g_ctrl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            next = g_ctrl;
            xSemaphoreGive(g_ctrl_mutex);
        }

        if (!parse_csv_command(buf, &next) && !parse_wasd_command(buf, &next)) {
            ESP_LOGW(TAG, "Bad packet: %s", buf);
            continue;
        }

        if (xSemaphoreTake(g_ctrl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_ctrl = next;
            xSemaphoreGive(g_ctrl_mutex);
        }
    }
}

static void control_task(void *arg) {
    (void)arg;
    control_state_t local;
    while (1) {
        memset(&local, 0, sizeof(local));
        if (xSemaphoreTake(g_ctrl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            local = g_ctrl;
            xSemaphoreGive(g_ctrl_mutex);
        }

        int64_t now_us = esp_timer_get_time();
        int timed_out = ((now_us - local.last_rx_us) / 1000) > CONTROL_TIMEOUT_MS;
        if (timed_out) {
            local.enable = 0;
            local.vx = 0.0f;
            local.vy = 0.0f;
            local.wz = 0.0f;
        }

        apply_mecanum(&local);
        set_elevation_servo(local.elevation_deg);

        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
}

void app_main(void) {
    nvs_flash_init();
    g_ctrl_mutex = xSemaphoreCreateMutex();
    g_ctrl.last_rx_us = esp_timer_get_time();

    init_pwm_and_gpio();
    wifi_init_softap();

    xTaskCreatePinnedToCore(udp_server_task, "udp_server", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(control_task, "control_task", 4096, NULL, 6, NULL, 1);

    ESP_LOGI(TAG, "Rover controller ready");
}
