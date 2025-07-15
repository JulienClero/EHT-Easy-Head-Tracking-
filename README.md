# EHT (Easy Head Tracking)

A DIY head tracking system using ESP32 and MPU6050 for gaming and simulations, compatible with OpenTrack.

## 🎯 Features

- **6DOF Tracking**: Yaw, Pitch, Roll with gyroscope and accelerometer
- **WiFi Transmission**: Real-time UDP data streaming
- **Complementary Filter**: Eliminates drift on pitch/roll
- **Low Latency**: Less than 20ms end-to-end
- **OpenTrack Compatible**: Plug & play with games and simulators

## 🔧 Required Components

### Current Version (v1.0)
- **ESP32** (DevKit v1 or similar) - ~$8
- **MPU6050** (accelerometer + gyroscope) - ~$3
- **4 jumper wires** female-female - ~$2
- **Case/mount** (3D printing recommended) - ~$2

### Future Version (v2.0) - In Development
- **ESP32** (DevKit v1 or similar) - ~$8
- **MPU6050** (accelerometer + gyroscope) - ~$3
- **Magnetometer** (HMC5883L/QMC5883L) - ~$4
- **EHT Custom PCB** (currently being designed) - ~$5
- **Li-Po Battery** (500-1000mAh) - ~$8
- **Charging Circuit** (TP4056 or integrated PCB) - ~$3
- **ON/OFF Switch** - ~$1
- **Custom Case** (3D printed) - ~$3

### Total Cost
- **v1.0 (wired)**: ~$15
- **v2.0 (wireless)**: ~$35

## 📐 Wiring Diagram

### Version v1.0 (Prototype)
```
ESP32          MPU6050
-----          -------
3.3V    ---    VCC
GND     ---    GND
GPIO21  ---    SDA
GPIO22  ---    SCL
```

### Version v2.0 (Custom PCB - Coming Soon)
```
ESP32          MPU6050         Magnetometer    Battery
-----          -------         ------------    -------
3.3V    ---    VCC      ---    VCC      ---    TP4056 OUT+
GND     ---    GND      ---    GND      ---    TP4056 OUT-
GPIO21  ---    SDA      ---    SDA
GPIO22  ---    SCL      ---    SCL
```
*Detailed schematic available with custom PCB*

## 🚀 Installation

### 1. Prepare Environment
```bash
# Install Arduino IDE
# Add ESP32 support:
# File > Preferences > Additional Board Manager URLs
# https://dl.espressif.com/dl/package_esp32_index.json
```

### 2. Configure Code
```cpp
// Modify these variables in the code:
const char* ssid     = "YOUR_WIFI";
const char* password = "YOUR_PASSWORD";
IPAddress pcIP(192, 168, 1, XXX);  // Your PC IP
```

### 3. Flash ESP32
- Select board: `ESP32 Dev Module`
- Select serial port
- Compile and upload

## 🎮 OpenTrack Configuration

### 1. Install OpenTrack
Download from: https://github.com/opentrack/opentrack/releases

### 2. Configuration
1. **Input**: `UDP over network`
   - Port: `4242`
   - Bind address: `0.0.0.0`

2. **Output**: Choose according to your game
   - `freetrack 2.0 Enhanced` (most games)
   - `TrackIR` (for compatible games)

3. **Mapping**: Adjust to your preferences
   - Yaw: ±180° → ±45°
   - Pitch: ±90° → ±30°
   - Roll: ±90° → ±20°

### 3. Calibration
- Start OpenTrack
- Click "Start"
- Calibrate neutral position with `Ctrl+Home`

## 📊 Performance

- **Frequency**: 100Hz (10ms per sample)
- **Total Latency**: < 20ms
- **Accuracy**: ±1° (pitch/roll), ±3° (yaw)
- **Drift**: None on pitch/roll, slight on yaw

## 🔧 Advanced Settings

### Sensitivity Adjustment
```cpp
// In code, final lines:
headData[0] = yaw * 2.0;   // Multiply for more sensitivity
headData[1] = pitch * 3.0;
headData[2] = roll * 3.0;
```

### Complementary Filter
```cpp
const float alpha = 0.99f;  // 0.95-0.99 recommended
// Closer to 1.0 = more responsive but more noise
// Closer to 0.9 = more stable but less responsive
```

## 🛠️ Troubleshooting

### WiFi Connection Issues
- Check SSID/password
- Verify ESP32 is on same network
- Restart router if necessary

### Angle Drift
- **Pitch/Roll**: No drift (complementary filter)
- **Yaw**: Normal drift without magnetometer
- Solution: Recalibrate with `Ctrl+Home` in OpenTrack

### Erratic Data
- Check I2C connections
- Add decoupling capacitors if needed
- Verify stable 3.3V power supply

### High Latency
- Reduce `LOOP_INTERVAL_MS` (watch CPU load)
- Use 5GHz WiFi if possible
- Disable WiFi sleep mode

## 📁 Project Structure

```
├── firmware/
│   ├── v1.0/
│   │   └── eht_v1.ino         # Main ESP32 code v1.0
│   └── v2.0/
│       └── eht_v2.ino         # Future code with magnetometer
├── hardware/              #Coming Soon
│   ├── pcb/
│   │   ├── eht_pcb.kicad_pro  # EHT Custom PCB (Fusion 360)
│   │   ├── gerber/            # Manufacturing files
│   │   └── bom.csv            # Bill of materials
│   └── 3d_models/
│       ├── case_v1/           # Prototype case
│       └── case_v2/           # Final case
├── docs/                  #Coming Soon
│   ├── assembly_guide.md      # Assembly guide
│   ├── calibration.md         # Calibration guide
│   └── troubleshooting.md     # Troubleshooting
└── README.md                  # This file
```

## 🤝 Contributing

Contributions are welcome! Feel free to:
- Report bugs
- Suggest improvements
- Share hardware modifications
- Add support for more games

## 📄 License

This project is licensed under the MIT License. See the `LICENSE` file for details.

## 🎮 Tested Games

- ✅ **DCS World** (freetrack)
- ✅ **Star Citizen** (freetrack)

## 🔮 Roadmap

### Version 1.0 (Current) ✅
- [x] 3DOF tracking (Yaw, Pitch, Roll)
- [x] WiFi UDP transmission
- [x] Complementary filter
- [x] OpenTrack compatibility

### Version 1.5 (In Progress)
- [ ] Code optimization
- [ ] Web interface for configuration
- [ ] Automatic calibration
- [ ] Multiple profile support

### Version 2.0 (Planned)
- [ ] **EHT Custom PCB** - Compact and professional design
- [ ] **Magnetometer support** - Eliminates yaw drift
- [ ] **Integrated Li-Po battery** - 8-12h autonomy
- [ ] **USB-C charging circuit** - Fast charging
- [ ] **Waterproof case** - Outdoor usage
- [ ] **Status LEDs** - Battery, WiFi, tracking

---

**⚠️ Important**: This project is for personal use. Please respect the terms of use of games and simulators.
