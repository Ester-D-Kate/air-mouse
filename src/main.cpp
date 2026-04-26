/**
 * ============================================================
 *  Air Mouse – ESP32 + MPU6500 + Bluetooth HID
 * ============================================================
 *
 *  Wiring
 *  ──────
 *  MPU6500  →  ESP32
 *  VCC      →  3.3V
 *  GND      →  GND
 *  SDA      →  GPIO 21
 *  SCL      →  GPIO 22
 *  AD0      →  GND   (I2C address = 0x68)
 *
 *  Left-click button  : GPIO 26 → GND  (internal pull-up)
 *  Right-click button : GPIO 27 → GND  (internal pull-up)
 *
 *  Library required (add to platformio.ini lib_deps):
 *    https://github.com/T-vK/ESP32-BLE-Mouse.git
 *
 *  Axis mapping (sensor chip facing up, USB port of dev-board down):
 *    Gyro Z  (yaw  – sweep left/right)  →  cursor X
 *    Gyro X  (pitch – tilt  up/down)    →  cursor Y
 *
 *  Tuning knobs:
 *    SENSITIVITY   – higher = faster cursor
 *    DEAD_ZONE_DPS – higher = more stillness needed before cursor moves
 *    LPF_ALPHA     – lower = smoother but laggier (0.1–0.5 is typical)
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <BleMouse.h>

// ═══════════════════════════════════════════════════════════════
//  Pin definitions
// ═══════════════════════════════════════════════════════════════
#define I2C_SDA             21
#define I2C_SCL             22
#define PIN_BTN_LEFT        26
#define PIN_BTN_RIGHT       27
#define LED_BUILTIN_PIN      2   // onboard LED – connection indicator

// ═══════════════════════════════════════════════════════════════
//  MPU6500 register map
// ═══════════════════════════════════════════════════════════════
#define MPU_ADDR            0x68   // AD0 LOW → 0x68 | AD0 HIGH → 0x69

#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_CONFIG2   0x1D
#define REG_INT_PIN_CFG     0x37
#define REG_INT_ENABLE      0x38
#define REG_GYRO_XOUT_H     0x43
#define REG_PWR_MGMT_1      0x6B
#define REG_PWR_MGMT_2      0x6C
#define REG_WHO_AM_I        0x75

// Gyro full-scale = ±500 dps → sensitivity = 65.5 LSB/(°/s)
#define GYRO_FS_SEL_500     0x08
#define GYRO_LSB_PER_DPS    65.5f

// ═══════════════════════════════════════════════════════════════
//  Tunable parameters
// ═══════════════════════════════════════════════════════════════
#define SENSITIVITY         15.0f   // deg/s → HID units (increase to move faster)
#define DEAD_ZONE_DPS        1.8f   // °/s threshold below which movement is ignored
#define LPF_ALPHA            0.25f  // low-pass filter weight (0=max smooth, 1=raw)
#define CALIB_SAMPLES        500    // samples taken at startup for bias removal
#define DEBOUNCE_MS           30    // button debounce period in ms
#define REPORT_INTERVAL_MS    10    // target HID report period (~100 Hz)

// ═══════════════════════════════════════════════════════════════
//  BLE Mouse instance
// ═══════════════════════════════════════════════════════════════
BleMouse bleMouse("Air Mouse", "ESP32", 100);

// ═══════════════════════════════════════════════════════════════
//  Global state
// ═══════════════════════════════════════════════════════════════
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

// Low-pass filter accumulators
float filtX = 0.0f, filtY = 0.0f, filtZ = 0.0f;

// Accumulator for sub-pixel mouse deltas (prevents quantization loss)
float accumX = 0.0f, accumY = 0.0f;

// Timing
uint32_t lastReportTime = 0;

// Button state
bool  leftWasPressed  = false;
bool  rightWasPressed = false;
uint32_t leftDebounceTime  = 0;
uint32_t rightDebounceTime = 0;

// ═══════════════════════════════════════════════════════════════
//  MPU6500 I2C helpers
// ═══════════════════════════════════════════════════════════════

static void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t mpuReadByte(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
    return Wire.available() ? Wire.read() : 0xFF;
}

static void mpuReadBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, len, (uint8_t)true);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        buf[i] = Wire.read();
    }
}

// ═══════════════════════════════════════════════════════════════
//  MPU6500 initialisation
// ═══════════════════════════════════════════════════════════════
static bool mpuInit() {
    uint8_t who = mpuReadByte(REG_WHO_AM_I);
    Serial.printf("[MPU] WHO_AM_I = 0x%02X\n", who);

    // 0x70 = MPU6500, 0x68 = MPU6050, 0x71 = MPU9250 (all share this driver)
    if (who != 0x70 && who != 0x68 && who != 0x71 && who != 0x19) {
        Serial.printf("[MPU] ERROR: unexpected WHO_AM_I (0x%02X). Check wiring.\n", who);
        return false;
    }

    mpuWrite(REG_PWR_MGMT_1, 0x80);    // full device reset
    delay(100);
    mpuWrite(REG_PWR_MGMT_1, 0x01);    // wake up; use PLL with X-axis gyro clock
    delay(10);
    mpuWrite(REG_PWR_MGMT_2, 0x00);    // enable accel + gyro (all axes)
    mpuWrite(REG_SMPLRT_DIV,  0x04);   // sample rate = gyro_rate / (1+4) = 200 Hz
    mpuWrite(REG_CONFIG,      0x03);   // DLPF BW ≈ 41 Hz, delay ≈ 5.9 ms
    mpuWrite(REG_GYRO_CONFIG, GYRO_FS_SEL_500);  // ±500 dps, no FCHOICE bypass
    mpuWrite(REG_ACCEL_CONFIG,  0x00); // accel ±2 g
    mpuWrite(REG_ACCEL_CONFIG2, 0x03); // accel DLPF 41 Hz

    Serial.println("[MPU] Initialised OK.");
    return true;
}

// ═══════════════════════════════════════════════════════════════
//  Gyro bias calibration (run while sensor is stationary)
// ═══════════════════════════════════════════════════════════════
static void calibrateGyro() {
    Serial.println("[CAL] Calibrating – keep sensor still for ~1 s ...");
    int64_t sx = 0, sy = 0, sz = 0;
    uint8_t buf[6];

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        mpuReadBurst(REG_GYRO_XOUT_H, buf, 6);
        sx += (int16_t)((buf[0] << 8) | buf[1]);
        sy += (int16_t)((buf[2] << 8) | buf[3]);
        sz += (int16_t)((buf[4] << 8) | buf[5]);
        delay(2);
    }

    gyroBiasX = (float)sx / CALIB_SAMPLES;
    gyroBiasY = (float)sy / CALIB_SAMPLES;
    gyroBiasZ = (float)sz / CALIB_SAMPLES;

    Serial.printf("[CAL] Bias raw LSB → X:%.1f  Y:%.1f  Z:%.1f\n",
                  gyroBiasX, gyroBiasY, gyroBiasZ);
    Serial.println("[CAL] Done.");
}

// ═══════════════════════════════════════════════════════════════
//  Read gyro → degrees/second (bias corrected)
// ═══════════════════════════════════════════════════════════════
static void readGyroDps(float &gx, float &gy, float &gz) {
    uint8_t buf[6];
    mpuReadBurst(REG_GYRO_XOUT_H, buf, 6);
    int16_t rx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ry = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t rz = (int16_t)((buf[4] << 8) | buf[5]);
    gx = (rx - gyroBiasX) / GYRO_LSB_PER_DPS;
    gy = (ry - gyroBiasY) / GYRO_LSB_PER_DPS;
    gz = (rz - gyroBiasZ) / GYRO_LSB_PER_DPS;
}

// ═══════════════════════════════════════════════════════════════
//  setup()
// ═══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n====  Air Mouse  ====");

    // Status LED
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN_PIN, LOW);

    // Buttons – active LOW with internal pull-up
    pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
    pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);

    // I2C @ 400 kHz
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    // MPU6500
    if (!mpuInit()) {
        Serial.println("[ERR] MPU6500 init failed. Halting.");
        while (true) {
            // Rapid blink = hardware fault
            digitalWrite(LED_BUILTIN_PIN, !digitalRead(LED_BUILTIN_PIN));
            delay(200);
        }
    }

    calibrateGyro();

    // BLE HID Mouse
    Serial.println("[BLE] Starting BLE Mouse...");
    bleMouse.begin();
    Serial.println("[BLE] Advertising. Pair with 'Air Mouse' on your host device.");

    lastReportTime = millis();
}

// ═══════════════════════════════════════════════════════════════
//  loop()
// ═══════════════════════════════════════════════════════════════
void loop() {
    uint32_t now = millis();

    // ── LED: blink slowly while waiting, solid when connected ──────────
    static uint32_t ledToggleTime = 0;
    if (!bleMouse.isConnected()) {
        if (now - ledToggleTime >= 500) {
            ledToggleTime = now;
            digitalWrite(LED_BUILTIN_PIN, !digitalRead(LED_BUILTIN_PIN));
        }
        // While disconnected, reset filter state so no stale values remain
        filtX = filtY = filtZ = 0.0f;
        accumX = accumY = 0.0f;
        delay(10);
        return;
    }
    // Connected
    digitalWrite(LED_BUILTIN_PIN, HIGH);

    // ── Compute actual dt since last report ─────────────────────────────
    float dt = (now - lastReportTime) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;   // guard against zero
    if (dt > 0.100f) dt = 0.100f;   // cap at 100 ms to prevent huge jumps

    // ── Read gyroscope ──────────────────────────────────────────────────
    float gx, gy, gz;
    readGyroDps(gx, gy, gz);

    // ── Low-pass filter (exponential moving average) ────────────────────
    filtX += LPF_ALPHA * (gx - filtX);
    filtY += LPF_ALPHA * (gy - filtY);
    filtZ += LPF_ALPHA * (gz - filtZ);

    // ── Dead zone: ignore small vibrations / sensor noise ───────────────
    float useZ = (fabsf(filtZ) > DEAD_ZONE_DPS) ? filtZ : 0.0f;  // yaw   → X
    float useX = (fabsf(filtX) > DEAD_ZONE_DPS) ? filtX : 0.0f;  // pitch → Y

    // ── Accumulate fractional pixels (prevents low-speed quantization) ──
    //    cursor X: positive gz = turn right  → positive dx
    //    cursor Y: positive gx = pitch up    → negative dy (screen Y is inverted)
    accumX += useZ * SENSITIVITY * dt;
    accumY += -useX * SENSITIVITY * dt;

    // Extract integer part and leave remainder in accumulator
    int8_t dx = (int8_t)constrain((int)accumX, -127, 127);
    int8_t dy = (int8_t)constrain((int)accumY, -127, 127);
    accumX -= dx;
    accumY -= dy;

    // ── Send mouse movement (only when there is something to report) ─────
    if (dx != 0 || dy != 0) {
        bleMouse.move(dx, dy, 0);
    }

    // ── Buttons (software-debounced, press & release) ────────────────────
    bool leftNow  = (digitalRead(PIN_BTN_LEFT)  == LOW);
    bool rightNow = (digitalRead(PIN_BTN_RIGHT) == LOW);

    // Left click
    if (leftNow != leftWasPressed && (now - leftDebounceTime) >= DEBOUNCE_MS) {
        leftDebounceTime = now;
        leftWasPressed   = leftNow;
        if (leftNow) bleMouse.press(MOUSE_LEFT);
        else         bleMouse.release(MOUSE_LEFT);
    }

    // Right click
    if (rightNow != rightWasPressed && (now - rightDebounceTime) >= DEBOUNCE_MS) {
        rightDebounceTime = now;
        rightWasPressed   = rightNow;
        if (rightNow) bleMouse.press(MOUSE_RIGHT);
        else          bleMouse.release(MOUSE_RIGHT);
    }

    // ── Rate-limit the loop to ~REPORT_INTERVAL_MS ──────────────────────
    lastReportTime = now;
    uint32_t elapsed = millis() - now;
    if (elapsed < REPORT_INTERVAL_MS) {
        delay(REPORT_INTERVAL_MS - elapsed);
    }
}
