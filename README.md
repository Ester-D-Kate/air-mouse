# Air Mouse — ESP32 + MPU6500 + Bluetooth HID

A wireless air mouse built with ESP32 and MPU6500 gyroscope. Control your computer's cursor by moving your hand in the air, with dedicated buttons for left/right click and a freeze button to reposition your hand.

---

## Requirements

### Hardware
- ESP32 development board (any standard ESP32, e.g., DevKitC)
- MPU6500 or MPU6050 gyroscope module
- 3 push buttons (for Freeze, Left Click, Right Click)
- Jumper wires, breadboard or PCB

### Software
- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- Arduino framework for ESP32

---

## Wiring

### MPU6500 Gyroscope

| MPU6500 Pin | ESP32 Pin | Notes |
|---|---|---|
| VCC | 3.3V | **Do NOT use 5V** |
| GND | GND | |
| SDA | GPIO 21 | |
| SCL | GPIO 22 | |
| AD0 | GND | (sets I2C address to 0x68) |

### Control Buttons

| Function | ESP32 Pin | Wiring |
|---|---|---|
| Freeze Button | GPIO 23 | One leg → GPIO 23, other leg → GND |
| Left Click | GPIO 19 | One leg → GPIO 19, other leg → GND |
| Right Click | GPIO 18 | One leg → GPIO 18, other leg → GND |

All buttons use **active LOW** — pressing a button connects the pin to GND. Internal pull-ups keep the pins HIGH when not pressed.

### Built-in LED
- GPIO 2 (most ESP32 boards have a built-in LED on this pin)

---

## Software Setup

### 1. Install PlatformIO
- **VS Code:** Install the "PlatformIO IDE" extension
- **CLI:** Run `pip install platformio`

### 2. Open the Project
```bash
cd air-mouse
pio run
```

### 3. Upload
```bash
pio run --target upload
```

### 4. Monitor Serial Output (Optional)
```bash
pio device monitor
```

---

## How It Works

### Controls

| Action | Effect |
|---|---|
| **D23 floating** (not pressed) | Cursor follows hand movement |
| **D23 pressed** to GND | Cursor freezes — reposition hand freely |
| **D19 pressed** | Left click |
| **D18 pressed** | Right click |

### Gyro-to-Cursor Mapping
- **Gyro Z (yaw)** → Cursor X (left/right)
- **Gyro X (pitch)** → Cursor Y (up/down)

The cursor moves only when the Freeze button is NOT pressed. This mimics lifting a physical mouse off the pad — you can reposition your hand freely without moving the cursor.

---

## Configuration

### Sensitivity

Find this line near the top of `src/main.cpp`:

```cpp
#define SENSITIVITY       8.0f
```

| Value | Feel |
|---|---|
| 3.0 | Very slow, precise cursor — for fine control |
| 8.0 | Default — good balance |
| 15.0 | Fast cursor — covers screen quickly |
| 20.0 | Very fast — for large displays |

To change: edit the number, save, and re-upload.

---

## LED Status

| LED Behaviour | Meaning |
|---|---|
| Slow blink (500 ms) | Advertising, waiting for computer to pair |
| Fast blink (100 ms) | Paired, waiting for HID setup to complete |
| Solid ON | Ready — cursor is active |

The LED will stay solid after the BLE connection is established and Windows has finished configuring the HID device.

---

## How to Pair

### First Time (or after removing the device)
1. Power on the ESP32
2. Open Windows Bluetooth settings
3. Click "Add a device"
4. Select "Air Mouse" from the list
5. Click "Connect" — do NOT click "Done" yet
6. Wait for LED to go from blinking to solid (~2 seconds)
7. Now the device is ready to use

### Subsequent Times
The ESP32 stores the pairing keys. On boot, it will automatically reconnect to Windows within a few seconds.

### If Connection Fails
- Open Windows Bluetooth settings
- Remove "Air Mouse" from the list
- Reboot the ESP32
- Pair again from step 2 above

---

## Troubleshooting

### "[MPU] Not found! Check wiring."
- Verify MPU6500 connections (SDA → GPIO 21, SCL → GPIO 22)
- Check VCC is 3.3V, not 5V
- Ensure AD0 is connected to GND

### LED stays in slow blink forever
- The ESP32 is advertising but not connecting
- Open Windows Bluetooth settings, verify "Air Mouse" is listed and connected
- If paired but LED still blinks: reboot the ESP32

### "[BLECharacteristic] notify(): rc=-1" errors in serial
- This is normal during the first 1-2 seconds after connection
- It happens because Windows hasn't subscribed to HID reports yet
- The code waits 1.5 seconds before sending to avoid this

### Cursor moves on its own (drift)
- The gyro has some zero-rate bias
- Keep the sensor still during the 1-second calibration at startup
- If drift persists, adjust `DEAD_ZONE` in the code (default: 2.0f)

### Cursor moves too fast/slow
- Adjust `SENSITIVITY` at the top of `src/main.cpp`

### Left/Right clicks not working
- Verify button wiring — each button should connect its GPIO pin to GND when pressed
- Check that the correct pin numbers match the defines: `PIN_LEFT = 19`, `PIN_RIGHT = 18`

---

## Dependencies

The project uses [ESP32-BLE-Mouse](https://github.com/T-vK/ESP32-BLE-Mouse) library, which is automatically installed by PlatformIO from `platformio.ini`:

```ini
lib_deps = https://github.com/T-vK/ESP32-BLE-Mouse.git
```

---

## File Structure

```
air-mouse/
├── platformio.ini      # Project configuration
├── src/
│   └── main.cpp      # Firmware source code
├── include/
│   └── README       # (placeholder)
├── lib/
│   └── README      # (placeholder)
└── test/
    └── README      # (placeholder)
```

---

## Credit

Built using the [ESP32-BLE-Mouse](https://github.com/T-vK/ESP32-BLE-Mouse) library by T-vK.