/**
 * ============================================================
 *  Air Mouse – ESP32 + MPU6500 + Bluetooth HID
 * ============================================================
 *
 *  ┌─────────────────────────────────────────────────────┐
 *  │  SENSITIVITY — change this number to tune speed     │
 *  │  3 = very slow/precise   8 = default   20 = fast    │
 *  └─────────────────────────────────────────────────────┘
 *
 *  HOW IT WORKS
 *  ─────────────
 *  D23 floating (not pressed) → cursor moves freely
 *  D23 to GND   (pressed)     → cursor freezes
 *  Press D19 → Left  click
 *  Press D18 → Right click
 *
 *  WIRING
 *  ──────
 *  MPU6500 VCC  →  3.3V
 *  MPU6500 GND  →  GND
 *  MPU6500 SDA  →  GPIO 21
 *  MPU6500 SCL  →  GPIO 22
 *  MPU6500 AD0  →  GND
 *
 *  Freeze button : one leg → GPIO 23, other leg → GND
 *  Left  button  : one leg → GPIO 19, other leg → GND
 *  Right button  : one leg → GPIO 18, other leg → GND
 *
 *  LED STATUS
 *  ──────────
 *  Slow blink (500 ms) = advertising, waiting to pair
 *  Fast blink (100 ms) = paired, finishing BLE setup
 *  Solid ON            = ready to use
 *
 *  LIBRARY NEEDED (platformio.ini lib_deps)
 *  ─────────────────────────────────────────
 *  https://github.com/T-vK/ESP32-BLE-Mouse.git
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <BleMouse.h>
#include <esp_log.h>     // lets us silence the harmless rc=-1 BLE log messages

// ─────────────────────────────────────────────────────────────
//  SENSITIVITY  ← CHANGE THIS NUMBER TO ADJUST CURSOR SPEED
//  3 = very slow / precise
//  8 = default (good starting point)
//  20 = very fast
// ─────────────────────────────────────────────────────────────
#define SENSITIVITY       20.0f

// ─────────────────────────────────────────────────────────────
//  PINS
// ─────────────────────────────────────────────────────────────
#define PIN_SDA          21
#define PIN_SCL          22
#define PIN_FREEZE       23   // Press (to GND) = freeze cursor | Release = cursor moves
#define PIN_LEFT         19   // Left click
#define PIN_RIGHT        18   // Right click
#define PIN_LED           2   // Built-in LED

// ─────────────────────────────────────────────────────────────
//  SETTINGS (you can leave these as-is)
// ─────────────────────────────────────────────────────────────
#define DEAD_ZONE         2.0f   // deg/s — gyro noise below this is ignored (prevents drift)
#define SMOOTHING         0.30f  // 0.1 = very smooth/slow, 0.9 = very raw/fast
#define DEBOUNCE_MS        30    // milliseconds — prevents button bounce
#define LOOP_MS            20    // milliseconds per loop tick (50 Hz report rate)
#define BLE_READY_MS     1500    // milliseconds to wait after connect before sending (Windows needs this)
#define CALIB_SAMPLES     500    // gyro samples to average at startup for drift removal

// ─────────────────────────────────────────────────────────────
//  MPU6500 REGISTERS
//  (don't touch these)
// ─────────────────────────────────────────────────────────────
#define MPU_ADDR   0x68
#define PWR1       0x6B
#define PWR2       0x6C
#define SMPLRT     0x19
#define DLPF_CFG   0x1A
#define GYRO_CFG   0x1B
#define ACC_CFG    0x1C
#define ACC_CFG2   0x1D
#define GYRO_OUT   0x43   // first of 6 bytes: XH XL YH YL ZH ZL
#define WHO_AM_I   0x75
#define GYRO_SCALE 65.5f  // LSB per °/s at ±500 dps full-scale

// ─────────────────────────────────────────────────────────────
//  BLE MOUSE
// ─────────────────────────────────────────────────────────────
BleMouse mouse("Air Mouse", "ESP32", 100);

// ─────────────────────────────────────────────────────────────
//  VARIABLES
// ─────────────────────────────────────────────────────────────
float biasX, biasY, biasZ;     // gyro zero-rate offset (measured at startup)
float filtX, filtY, filtZ;     // smoothed gyro values
float accumX, accumY;           // sub-pixel accumulator (keeps fractional pixels between ticks)

bool     leftPressed   = false;
bool     rightPressed  = false;
uint32_t leftDebTime   = 0;
uint32_t rightDebTime  = 0;

bool     bleConnected  = false;
bool     bleReady      = false;
uint32_t connectedAt   = 0;
uint32_t lastLoopTime  = 0;

// ─────────────────────────────────────────────────────────────
//  MPU6500 COMMUNICATION
// ─────────────────────────────────────────────────────────────

void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t mpuReadByte(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
    return Wire.available() ? Wire.read() : 0xFF;
}

void mpuReadSix(uint8_t reg, uint8_t *buf) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);
    for (uint8_t i = 0; i < 6 && Wire.available(); i++) buf[i] = Wire.read();
}

// ─────────────────────────────────────────────────────────────
//  MPU6500 INITIALISE
// ─────────────────────────────────────────────────────────────

bool mpuInit() {
    uint8_t id = mpuReadByte(WHO_AM_I);
    Serial.printf("[MPU] WHO_AM_I = 0x%02X\n", id);
    // 0x68 = MPU6050, 0x70 = MPU6500, 0x71 = MPU9250 — all work here
    if (id != 0x68 && id != 0x70 && id != 0x71) {
        Serial.println("[MPU] Not found! Check wiring.");
        return false;
    }
    mpuWrite(PWR1,     0x80); delay(100); // full reset
    mpuWrite(PWR1,     0x01); delay(10);  // wake up, use gyro clock
    mpuWrite(PWR2,     0x00);             // enable gyro + accel
    mpuWrite(SMPLRT,   0x04);             // 200 Hz sample rate
    mpuWrite(DLPF_CFG, 0x03);             // 41 Hz low-pass filter
    mpuWrite(GYRO_CFG, 0x08);             // ±500 dps range
    mpuWrite(ACC_CFG,  0x00);             // ±2g range
    mpuWrite(ACC_CFG2, 0x03);             // accel low-pass filter
    Serial.println("[MPU] OK");
    return true;
}

// ─────────────────────────────────────────────────────────────
//  GYRO CALIBRATION
//  Keep the sensor perfectly still during the first ~1 second
// ─────────────────────────────────────────────────────────────

void calibrate() {
    Serial.println("[CAL] Keep sensor still for 1 second...");
    int64_t sx = 0, sy = 0, sz = 0;
    uint8_t buf[6];
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        mpuReadSix(GYRO_OUT, buf);
        sx += (int16_t)((buf[0] << 8) | buf[1]);
        sy += (int16_t)((buf[2] << 8) | buf[3]);
        sz += (int16_t)((buf[4] << 8) | buf[5]);
        delay(2);
    }
    biasX = (float)sx / CALIB_SAMPLES;
    biasY = (float)sy / CALIB_SAMPLES;
    biasZ = (float)sz / CALIB_SAMPLES;
    Serial.printf("[CAL] Bias  X:%.1f  Y:%.1f  Z:%.1f\n", biasX, biasY, biasZ);
    Serial.println("[CAL] Done.");
}

// ─────────────────────────────────────────────────────────────
//  READ GYRO  →  degrees per second, bias removed
// ─────────────────────────────────────────────────────────────

void readGyro(float &gx, float &gy, float &gz) {
    uint8_t buf[6];
    mpuReadSix(GYRO_OUT, buf);
    gx = ((int16_t)((buf[0] << 8) | buf[1]) - biasX) / GYRO_SCALE;
    gy = ((int16_t)((buf[2] << 8) | buf[3]) - biasY) / GYRO_SCALE;
    gz = ((int16_t)((buf[4] << 8) | buf[5]) - biasZ) / GYRO_SCALE;
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Air Mouse ===");

    // LED
    pinMode(PIN_LED,   OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // Buttons (active LOW — press connects pin to GND)
    pinMode(PIN_FREEZE, INPUT_PULLUP);
    pinMode(PIN_LEFT,   INPUT_PULLUP);
    pinMode(PIN_RIGHT,  INPUT_PULLUP);

    // Silence the "rc=-1" BLE log message — it is harmless, just spammy
    esp_log_level_set("BLECharacteristic", ESP_LOG_NONE);

    // I2C at 400 kHz
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    // Start MPU6500
    if (!mpuInit()) {
        // Rapid blink = hardware error, halted
        while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(200); }
    }

    // Calibrate gyro drift
    calibrate();

    // Start BLE mouse
    Serial.println("[BLE] Starting...");
    mouse.begin();
    Serial.println("[BLE] Advertising as 'Air Mouse'");

    lastLoopTime = millis();
}

// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────

void loop() {
    uint32_t now = millis();
    static uint32_t ledBlink = 0;

    bool connected = mouse.isConnected();

    // ── Detect connect / disconnect ──────────────────────────────────────────
    if (connected && !bleConnected) {
        bleConnected = true;
        bleReady     = false;
        connectedAt  = now;
        // Reset state on fresh connect
        filtX = filtY = filtZ = 0;
        accumX = accumY = 0;
        leftPressed = rightPressed = false;
        Serial.println("[BLE] Connected — waiting for Windows HID setup...");
    }
    if (!connected && bleConnected) {
        bleConnected = false;
        bleReady     = false;
        Serial.println("[BLE] Disconnected.");
    }

    // ── Not connected: slow blink, wait ─────────────────────────────────────
    if (!connected) {
        if (now - ledBlink >= 500) {
            ledBlink = now;
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        }
        filtX = filtY = filtZ = 0;
        accumX = accumY = 0;
        delay(10);
        return;
    }

    // ── Just connected: fast blink for BLE_READY_MS, then go live ───────────
    // Windows needs this time to discover services and subscribe to HID reports.
    // Sending reports before it's ready floods Serial with rc=-1 errors.
    if (!bleReady) {
        if (now - connectedAt < BLE_READY_MS) {
            if (now - ledBlink >= 100) {
                ledBlink = now;
                digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            }
            delay(10);
            return;
        }
        bleReady     = true;
        lastLoopTime = now;
        Serial.println("[BLE] Ready!");
    }

    // ── Solid LED = live ─────────────────────────────────────────────────────
    digitalWrite(PIN_LED, HIGH);

    // ── Read gyro ────────────────────────────────────────────────────────────
    float gx, gy, gz;
    readGyro(gx, gy, gz);

    // ── Smooth the gyro reading (reduces jitter) ─────────────────────────────
    filtX += SMOOTHING * (gx - filtX);
    filtY += SMOOTHING * (gy - filtY);
    filtZ += SMOOTHING * (gz - filtZ);

    // ── Freeze button (D23) ──────────────────────────────────────────────────
    // D23 floating (not pressed) → cursor moves freely
    // D23 to GND   (pressed)     → cursor freezes (reposition hand freely)
    bool frozen = (digitalRead(PIN_FREEZE) == LOW);

    if (!frozen) {
        // Apply dead zone — ignore tiny gyro noise below threshold
        float vX = (fabsf(filtZ) > DEAD_ZONE) ? filtZ : 0.0f;   // yaw   → left/right
        float vY = (fabsf(filtX) > DEAD_ZONE) ? filtX : 0.0f;   // pitch → up/down

        // Calculate how many pixels to move this tick
        // dt = real time since last tick in seconds
        float dt = constrain((now - lastLoopTime) / 1000.0f, 0.001f, 0.1f);

        // Negative signs correct the direction (confirmed working)
        accumX += -vX * SENSITIVITY * dt;
        accumY += -vY * SENSITIVITY * dt;
    } else {
        // Frozen — flush accumulator so no jump when released
        accumX = 0.0f;
        accumY = 0.0f;
    }

    // Extract whole pixels, keep the fraction for the next tick
    int8_t dx = (int8_t)constrain((int)accumX, -127, 127);
    int8_t dy = (int8_t)constrain((int)accumY, -127, 127);
    accumX -= dx;
    accumY -= dy;

    if (dx != 0 || dy != 0) {
        mouse.move(dx, dy, 0);
    }

    // ── Left click (D19) ─────────────────────────────────────────────────────
    bool leftNow = (digitalRead(PIN_LEFT) == LOW);
    if (leftNow != leftPressed && (now - leftDebTime) >= DEBOUNCE_MS) {
        leftDebTime = now;
        leftPressed = leftNow;
        if (leftNow) mouse.press(MOUSE_LEFT);
        else         mouse.release(MOUSE_LEFT);
    }

    // ── Right click (D18) ────────────────────────────────────────────────────
    bool rightNow = (digitalRead(PIN_RIGHT) == LOW);
    if (rightNow != rightPressed && (now - rightDebTime) >= DEBOUNCE_MS) {
        rightDebTime = now;
        rightPressed  = rightNow;
        if (rightNow) mouse.press(MOUSE_RIGHT);
        else          mouse.release(MOUSE_RIGHT);
    }

    // ── Hold loop to LOOP_MS tick rate ───────────────────────────────────────
    lastLoopTime = now;
    uint32_t took = millis() - now;
    if (took < LOOP_MS) delay(LOOP_MS - took);
}
