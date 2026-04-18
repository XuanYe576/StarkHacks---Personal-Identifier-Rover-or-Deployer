// ============================================================================
//  StarkHacks ESP32 CSI Receiver Firmware v2.0
//  WiFi Channel State Information collector with MAC address identification
//  Non-blocking architecture with servo control for antenna positioning
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>

// ====== USER CONFIGURATION ======
#define WIFI_SSID            "YourSSID"
#define WIFI_PASS            "YourPassword"
#define CSI_OUTPUT_BAUD      921600
#define SERVO_ELEV_PIN       18
#define SERVO_AZIM_PIN       19
#define SERVO_RATE_LIMIT_MS  20    // min ms between servo updates
#define CSI_RING_SIZE        16
#define CSI_FRAME_MAX_BINS   128
// ================================

#define FIRMWARE_VERSION     "StarkHacks-CSIv2.0"

// ---- Servo objects ----
Servo servo_elev;
Servo servo_azim;

// ---- Servo state ----
float servo_elev_target   = 90.0f;
float servo_elev_current  = 90.0f;
float servo_azim_target   = 90.0f;
float servo_azim_current  = 90.0f;
unsigned long servo_last_update_ms = 0;

// ---- MAC Address ----
char device_mac_str[18];   // "xx:xx:xx:xx:xx:xx\0"

// ---- Sequence counter ----
volatile uint32_t csi_seq_counter = 0;

// ---- CSI Ring Buffer ----
struct CSIFrame {
    uint32_t seq;
    int8_t   rssi;
    uint16_t raw_len;
    uint16_t bin_count;
    uint8_t  mac[6];                 // sender MAC
    float    amplitude[CSI_FRAME_MAX_BINS];
    float    phase[CSI_FRAME_MAX_BINS];
    bool     valid = false;
};

static CSIFrame csi_ring[CSI_RING_SIZE];
static volatile uint8_t csi_ring_head = 0;
static volatile uint8_t csi_ring_tail = 0;
static volatile uint8_t csi_ring_count = 0;
static portMUX_TYPE csi_spinlock = portMUX_INITIALIZER_UNLOCKED;

// ---- Serial command buffer ----
static char serial_buf[64];
static uint8_t serial_idx = 0;

// ============================================================================
//  Helper: Format MAC bytes to xx:xx:xx:xx:xx:xx string
// ============================================================================
static void format_mac(const uint8_t *mac, char *out) {
    snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ============================================================================
//  CSI Parsing Helpers
// ============================================================================

/*
 * ESP32 CSI data is stored as signed bytes (int8_t).
 * For each subcarrier there are two bytes: [imag, real].
 * Amplitude = sqrt(real^2 + imag^2)
 * Phase     = atan2(imag, real)  -> in radians
 *
 * Subcarrier layout depends on channel bandwidth and LTF type:
 *   LLTF (20 MHz):  52 data/pilot subcarriers + 4 null subcarriers = 64 total bins
 *                   (indexed -28..+28, skipping 0, DC)
 *   HT-LTF (20 MHz): 56 data/pilot + 4 null = 60 effective bins
 *                    Actually: 56 data/pilot + 8 null = 64, some sources say 114 for 40MHz
 *   HT-LTF (40 MHz): 114 data/pilot + 6 null = 120 effective bins
 *
 * We extract as many bins as available, up to CSI_FRAME_MAX_BINS.
 */
static int parse_csi_subcarriers(const wifi_csi_info_t *info,
                                 float *amplitude,
                                 float *phase,
                                 int max_bins)
{
    if (!info || !info->buf || info->len == 0) return 0;

    const int8_t *csi_buf = info->buf;
    int bin_count = 0;
    int total_pairs = info->len / 2;

    for (int i = 0; i < total_pairs && bin_count < max_bins; i++) {
        int8_t imag = csi_buf[i * 2 + 0];
        int8_t real = csi_buf[i * 2 + 1];

        float re = (float)real;
        float im = (float)imag;

        amplitude[bin_count] = sqrtf(re * re + im * im);
        phase[bin_count]     = atan2f(im, re);  // radians
        bin_count++;
    }

    return bin_count;
}

// ============================================================================
//  CSI Receive Callback (runs in WiFi task context)
// ============================================================================
static void IRAM_ATTR csi_rx_callback(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;
    if (!info || !info->buf) return;

    // Quick reject if ring buffer is full
    if (csi_ring_count >= CSI_RING_SIZE) return;

    uint8_t slot = csi_ring_head;
    CSIFrame *frame = &csi_ring[slot];

    // Extract subcarriers
    int bins = parse_csi_subcarriers(info,
                                     frame->amplitude,
                                     frame->phase,
                                     CSI_FRAME_MAX_BINS);
    if (bins <= 0) return;

    // Safely increment sequence counter
    uint32_t seq = __atomic_add_fetch(&csi_seq_counter, 1, __ATOMIC_RELAXED);

    frame->seq       = seq;
    frame->rssi      = info->rx_ctrl.rssi;
    frame->raw_len   = info->len;
    frame->bin_count = (uint16_t)bins;

    // Copy sender MAC address
    memcpy(frame->mac, info->mac, 6);

    // Store MAC of the frame source (the AP we are connected to, or other STA)
    // info->mac contains the MAC address of the device that sent the packet

    // Mark valid and advance head
    portENTER_CRITICAL_ISR(&csi_spinlock);
    frame->valid = true;
    csi_ring_head = (csi_ring_head + 1) & (CSI_RING_SIZE - 1);
    csi_ring_count++;
    portEXIT_CRITICAL_ISR(&csi_spinlock);
}

// ============================================================================
//  Print a single CSIv2 frame to Serial
// ============================================================================
static void print_csi_frame_v2(const CSIFrame *f)
{
    if (!f || !f->valid) return;

    // Build MAC string for this frame
    char mac_str[18];
    format_mac(f->mac, mac_str);

    // Header: CSIv2,<seq>,<macaddr>,<rssi>,<raw_len>,<bins>
    Serial.printf("CSIv2,%lu,%s,%d,%u,%u",
                  (unsigned long)f->seq,
                  mac_str,
                  f->rssi,
                  f->raw_len,
                  f->bin_count);

    // Amplitudes
    Serial.print(",A");
    for (uint16_t i = 0; i < f->bin_count; i++) {
        Serial.printf(",%.3f", f->amplitude[i]);
    }

    // Phases
    Serial.print(",P");
    for (uint16_t i = 0; i < f->bin_count; i++) {
        Serial.printf(",%.3f", f->phase[i]);
    }

    Serial.println();  // newline terminator
}

// ============================================================================
//  Drain ring buffer and print CSI frames
// ============================================================================
static void drain_csi_ring(void)
{
    while (true) {
        portENTER_CRITICAL(&csi_spinlock);
        if (csi_ring_count == 0) {
            portEXIT_CRITICAL(&csi_spinlock);
            break;
        }
        uint8_t slot = csi_ring_tail;
        CSIFrame frame_copy = csi_ring[slot];
        csi_ring[slot].valid = false;
        csi_ring_tail = (csi_ring_tail + 1) & (CSI_RING_SIZE - 1);
        csi_ring_count--;
        portEXIT_CRITICAL(&csi_spinlock);

        print_csi_frame_v2(&frame_copy);
    }
}

// ============================================================================
//  WiFi + CSI Setup
// ============================================================================
static void setup_wifi_csi(void)
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print("[WiFi] Connecting to ");
    Serial.print(WIFI_SSID);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 60) {
        delay(500);
        Serial.print('.');
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" CONNECTED");
        Serial.print("[WiFi] IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println(" FAILED - will retry in loop");
    }

    // ---- Configure CSI ----
    wifi_csi_config_t csi_config = {
        .lltf_en         = true,   // enable legacy LTF CSI
        .htltf_en        = true,   // enable HT-LTF CSI
        .stbc_htltf2_en  = true,   // enable STBC HT-LTF2 CSI
        .manu_scale      = false,  // automatic scaling
    };

    esp_err_t err = esp_wifi_set_csi_config(&csi_config);
    if (err != ESP_OK) {
        Serial.printf("[CSI] set_csi_config failed: %d\n", (int)err);
    }

    err = esp_wifi_set_csi_rx_cb(csi_rx_callback, NULL);
    if (err != ESP_OK) {
        Serial.printf("[CSI] set_csi_rx_cb failed: %d\n", (int)err);
    }

    err = esp_wifi_set_csi(true);
    if (err != ESP_OK) {
        Serial.printf("[CSI] set_csi(true) failed: %d\n", (int)err);
    } else {
        Serial.println("[CSI] CSI collection enabled");
    }
}

// ============================================================================
//  Reconnect WiFi if needed (non-blocking)
// ============================================================================
static unsigned long wifi_last_check_ms = 0;

static void maintain_wifi(void)
{
    unsigned long now = millis();
    if (now - wifi_last_check_ms < 5000) return;
    wifi_last_check_ms = now;

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Reconnecting...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
}

// ============================================================================
//  Servo Setup
// ============================================================================
static void setup_servos(void)
{
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    servo_elev.setPeriodHertz(50);  // standard servo 50Hz
    servo_azim.setPeriodHertz(50);

    servo_elev.attach(SERVO_ELEV_PIN, 500, 2400);  // 500-2400us pulse width
    servo_azim.attach(SERVO_AZIM_PIN, 500, 2400);

    servo_elev.write(90);
    servo_azim.write(90);

    Serial.println("[Servo] Elevation on GPIO " + String(SERVO_ELEV_PIN));
    Serial.println("[Servo] Azimuth   on GPIO " + String(SERVO_AZIM_PIN));
}

// ============================================================================
//  Smooth servo update (rate-limited, non-blocking)
//  Max 5 degrees per SERVO_RATE_LIMIT_MS ms
// ============================================================================
static void update_servos(void)
{
    unsigned long now = millis();
    if ((long)(now - servo_last_update_ms) < (long)SERVO_RATE_LIMIT_MS) return;
    servo_last_update_ms = now;

    const float max_step = 5.0f;  // max degrees per update

    // Elevation
    float delta_e = servo_elev_target - servo_elev_current;
    if (delta_e > max_step)       delta_e = max_step;
    else if (delta_e < -max_step) delta_e = -max_step;
    servo_elev_current += delta_e;
    servo_elev.write((int)(servo_elev_current + 0.5f));

    // Azimuth
    float delta_a = servo_azim_target - servo_azim_current;
    if (delta_a > max_step)       delta_a = max_step;
    else if (delta_a < -max_step) delta_a = -max_step;
    servo_azim_current += delta_a;
    servo_azim.write((int)(servo_azim_current + 0.5f));
}

// ============================================================================
//  Serial Command Parser
// ============================================================================
static void process_serial_command(const char *cmd)
{
    if (cmd == nullptr || cmd[0] == '\0') return;

    char c = cmd[0];

    switch (c) {
        // ---- Elevation servo ----
        case 'E':
        case 'e': {
            int angle = atoi(&cmd[1]);
            if (angle < 0)   angle = 0;
            if (angle > 180) angle = 180;
            servo_elev_target = (float)angle;
            Serial.printf("[CMD] Elevation target: %d deg\n", angle);
            break;
        }

        // ---- Azimuth servo ----
        case 'A':
        case 'a': {
            // Check if it's the "A" servo command or something else
            if (cmd[1] >= '0' && cmd[1] <= '9') {
                int angle = atoi(&cmd[1]);
                if (angle < 0)   angle = 0;
                if (angle > 180) angle = 180;
                servo_azim_target = (float)angle;
                Serial.printf("[CMD] Azimuth target: %d deg\n", angle);
            } else if (cmd[1] == '\0' || cmd[1] == '\r' || cmd[1] == '\n') {
                // Just 'A' with number - already handled above
                Serial.println("[CMD] Azimuth: missing angle");
            }
            break;
        }

        // ---- Reset sequence counter ----
        case 'R':
        case 'r': {
            csi_seq_counter = 0;
            Serial.println("[CMD] Sequence counter reset to 0");
            break;
        }

        // ---- Print device info ----
        case 'I':
        case 'i': {
            Serial.println("===== Device Info =====");
            Serial.printf("Firmware: %s\n", FIRMWARE_VERSION);
            Serial.printf("MAC:      %s\n", device_mac_str);
            Serial.printf("WiFi:     %s\n", WIFI_SSID);
            Serial.printf("RSSI:     %d dBm\n", WiFi.RSSI());
            Serial.printf("IP:       %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("CSI baud: %lu\n", (unsigned long)CSI_OUTPUT_BAUD);
            Serial.printf("Servo E:  GPIO %d (%.0f deg)\n", SERVO_ELEV_PIN, servo_elev_current);
            Serial.printf("Servo A:  GPIO %d (%.0f deg)\n", SERVO_AZIM_PIN, servo_azim_current);
            Serial.printf("CSI ring: %d slots\n", CSI_RING_SIZE);
            Serial.println("=======================");
            break;
        }

        // ---- Re-read and print MAC ----
        case 'M':
        case 'm': {
            uint8_t mac[6];
            esp_read_mac(mac, ESP_MAC_WIFI_STA);
            format_mac(mac, device_mac_str);
            Serial.printf("[CMD] MAC: %s\n", device_mac_str);
            break;
        }

        default:
            Serial.printf("[CMD] Unknown: '%c'\n", c);
            break;
    }
}

// ============================================================================
//  Non-blocking serial read
// ============================================================================
static void handle_serial_input(void)
{
    while (Serial.available()) {
        char c = Serial.read();

        // Line termination
        if (c == '\n' || c == '\r') {
            if (serial_idx > 0) {
                serial_buf[serial_idx] = '\0';
                process_serial_command(serial_buf);
                serial_idx = 0;
            }
        } else if (serial_idx < sizeof(serial_buf) - 1) {
            serial_buf[serial_idx++] = c;
        }
    }
}

// ============================================================================
//  Setup
// ============================================================================
void setup()
{
    // Disable brownout detector to prevent reset during WiFi init
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(CSI_OUTPUT_BAUD);
    while (!Serial) { ; }  // wait for native USB Serial

    delay(100);
    Serial.println();
    Serial.println("========================================");
    Serial.printf("  %s\n", FIRMWARE_VERSION);
    Serial.println("========================================");

    // ---- Read factory MAC address ----
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    format_mac(mac, device_mac_str);

    Serial.print("[MAC] Device STA MAC: ");
    Serial.println(device_mac_str);

    // ---- Setup subsystems ----
    setup_servos();
    setup_wifi_csi();

    Serial.println("[INIT] Ready. Commands: E<angle>, A<angle>, R, I, M");
}

// ============================================================================
//  Main Loop (non-blocking)
// ============================================================================
void loop()
{
    handle_serial_input();
    update_servos();
    maintain_wifi();
    drain_csi_ring();
}
