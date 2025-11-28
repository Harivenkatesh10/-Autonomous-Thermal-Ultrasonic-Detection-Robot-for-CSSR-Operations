# CSSR: Thermal & Ultrasonic Enhanced Detection System for Collapsed Structure Search and Rescue

## 📋 Project Overview

This project presents an **autonomous robotic system** designed to enhance search and rescue (SAR) operations in collapsed structures. The robot integrates thermal imaging and ultrasonic sensing to detect trapped individuals and navigate hazardous debris environments autonomously, reducing manual search efforts and improving detection accuracy.

**Key Application**: Locating and identifying victims trapped under debris during disaster response operations.

---

## 🎯 Objectives

- Develop an affordable, autonomous robotic platform for CSSR operations
- Integrate multi-sensor fusion (thermal + ultrasonic) for robust victim detection
- Implement autonomous navigation with obstacle avoidance
- Enable real-time remote monitoring via Bluetooth/WiFi
- Reduce search time and improve rescue team safety

---

## 🔧 Hardware Components

| Component | Specification | Purpose |
|-----------|---------------|---------|
| **ESP32 Microcontroller** | Dual-core, 240 MHz, WiFi/Bluetooth | Core processing, sensor data, motor control |
| **Thermal Sensor** | MLX90614 (I2C) | Non-contact body heat detection (35–38°C range) |
| **Ultrasonic Sensors** | HC-SR04 (GPIO) | Obstacle detection and distance mapping |
| **Motor Driver** | L298N (H-bridge) | PWM-controlled DC motor driving |
| **DC Motors** | 2× 12V motors with wheels | Robot locomotion and steering |
| **Chassis** | 4-wheel platform | Durable, terrain-traversable base |
| **Power Supply** | 7.4V Li-ion rechargeable battery | ~1 hour runtime |
| **Communication** | Built-in ESP32 WiFi/Bluetooth | Wireless data transmission |
| **Alerts** | Buzzer + LED | Heat signature indicators |

---

## 📡 System Architecture

### **Hardware Connections**

```
ESP32 Pins:
├── GPIO 26, 27 (LEFT motor: LIN1, LIN2)
├── GPIO 14, 4  (RIGHT motor: RIN1, RIN2)
├── GPIO 25, 33 (PWM: ENA, ENB via ledc)
├── GPIO 13, 34 (Ultrasonic: TRIG_F, ECHO_F)
├── GPIO 23, 19 (Alerts: BUZZER, LED)
├── I2C (GPIO 21, 22) → MLX90614 thermal sensor
└── Bluetooth Serial (default UART)
```

### **Detection Logic**

```
If (Temperature ∈ [35°C, 38°C]) AND (Distance ≤ 300 cm)
  → Sound buzzer (5 beeps)
  → Light LED
  → Transmit alert via Bluetooth
  → Increment body counter
  → Pause for 2 seconds
```

### **Navigation Modes**

| Mode | Behavior |
|------|----------|
| **Autonomous** | Grid search pattern; turns left/right based on timing; pauses when obstacle detected |
| **Manual** | Commands via Bluetooth: W (forward), A (left), D (right), S (backward), X (stop), E (toggle mode) |

---

## 💻 Software Architecture

### **Key Algorithms**

1. **Thermal Detection**: Continuous temperature monitoring with threshold-based alerting
2. **Obstacle Avoidance**: Reactive control—reverse and turn when obstacle detected
3. **Grid Search**: Time-based navigation pattern for systematic area coverage
4. **Data Logging**: Status updates every 2 seconds (temperature, distance, mode, bodies found)
5. **Motor Control**: PWM-based speed regulation (0–255 levels) for precise movement

### **Control Flow**

```
Setup()
  ├─ Initialize ESP32 pins
  ├─ Configure PWM timers
  ├─ Start Bluetooth serial
  ├─ Calibrate thermal sensor
  └─ Startup buzzer alert

Loop()
  ├─ Read thermal temperature
  ├─ Read ultrasonic distance
  ├─ Check for body heat detection
  ├─ Process manual/Bluetooth commands
  ├─ Execute autonomous or manual mode
  └─ Log status every 2 seconds
```

---

## 🚀 Getting Started

### **Prerequisites**

- Arduino IDE (v1.8+)
- ESP32 Board Manager installed
- Libraries:
  - `Adafruit MLX90614` (thermal sensor)
  - `BluetoothSerial` (included with ESP32 core)
  - `Wire.h` (I2C, included)

### **Installation**

1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/CSSR-Robot.git
   cd CSSR-Robot
   ```

2. **Install Arduino IDE** and add ESP32 support:
   - Go to `Preferences` → `Additional Board Manager URLs`
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`
   - Install `esp32` from Board Manager

3. **Install required libraries**:
   - Open Arduino IDE → `Sketch` → `Include Library` → `Manage Libraries`
   - Search and install: **Adafruit MLX90614**

4. **Upload firmware**:
   - Open `CSSR-code.ino` in Arduino IDE
   - Select Board: `ESP32 Dev Module`
   - Select COM Port: (your device's serial port)
   - Click `Upload`

---

## 📊 Configuration Parameters

Edit these in `CSSR-code.ino` to tune behavior:

```c
// Temperature thresholds (Celsius)
const float TEMP_MIN = 35.0;  // Minimum body temperature
const float TEMP_MAX = 38.0;  // Maximum body temperature

// Distance threshold (cm)
const int OBSTACLE_DIST = 30;  // Stop if obstacle closer than this

// Motor speed (0–255 PWM)
const int SPEED = 180;

// Autonomous mode timing
// Phase 0: Forward 2500ms
// Phase 1: Turn left 600ms
// Phase 2: Forward 2500ms
// Phase 3: Turn right 600ms
// → Repeats
```

---

## 📱 Operation

### **Bluetooth Commands** (via serial monitor or mobile app)

| Command | Action |
|---------|--------|
| **W** | Move forward |
| **A** | Turn left |
| **D** | Turn right |
| **S** | Move backward |
| **X** | Stop |
| **E** | Toggle between auto/manual mode |

### **Running the Robot**

1. **Power on** the ESP32 (battery or USB)
2. **Open Arduino Serial Monitor** (115200 baud)
3. **Connect via Bluetooth** (device name: "CSSR_Robot")
4. **Send commands** (or press 'E' to enter autonomous mode)
5. **Monitor output** for temperature readings and body detections

### **Sample Output**

```
CSSR ROBOT - SIMPLE
Autonomous Thermal Search
Mode: AUTO
Commands: WASDXE

Temp: 28.5°C | Dist: 156 cm | Mode: AUTO | Found: 0

[OBSTACLE DETECTED]
Reversing... Turning...

Temp: 36.2°C | Dist: 45 cm | Mode: AUTO | Found: 0

[BODY DETECTED!]
Detection #1
Temperature: 36.2°C
Distance: 45 cm
Buzzer: 5 beeps
```

---

## 🧪 Testing & Calibration

### **Thermal Sensor Calibration**

1. Place a heated object (e.g., water bottle at 37°C) near the robot
2. Verify MLX90614 reading via serial monitor
3. Adjust `TEMP_MIN` and `TEMP_MAX` if needed

### **Ultrasonic Sensor Calibration**

1. Position obstacles at known distances (10, 20, 30, 50 cm)
2. Check serial output distance readings
3. Apply smoothing filter if readings are noisy

### **Field Simulation Testing**

- Set up a rubble mock-up (cardboard boxes, concrete blocks)
- Place heated water bottles at various depths
- Run autonomous mode and measure detection accuracy, navigation speed, and battery drain

---

## 📈 Performance Metrics

| Metric | Target | Achieved* |
|--------|--------|-----------|
| **Body Detection Range** | 5–10 meters | ✓ (thermal sensor limit) |
| **Temperature Accuracy** | ±1.5°C | ✓ (MLX90614 spec) |
| **Obstacle Detection Range** | 2–400 cm | ✓ (HC-SR04 spec) |
| **Battery Runtime** | 1 hour continuous | ✓ (~60 min) |
| **Max Navigation Speed** | ~30 cm/s | ✓ (tunable via SPEED) |
| **Response Latency** | <500 ms | ✓ (alert to buzzer) |

*Based on controlled lab testing. Field performance may vary with debris complexity.

---

## ⚠️ Limitations & Future Work

### **Current Limitations**

- Single front-facing thermal sensor (no 3D temperature mapping)
- Fixed grid search pattern (doesn't adapt to terrain)
- No GPS/IMU (limited localization accuracy)
- ~1 hour battery life (requires frequent recharging)
- False positives from non-human heat sources

### **Future Enhancements**

1. **Multi-Sensor Fusion**: Add side ultrasonic sensors for improved 3D obstacle awareness
2. **SLAM Integration**: Implement occupancy grid mapping with odometry
3. **Machine Learning**: Train neural network to classify body vs. debris heat signatures
4. **Extended Runtime**: Optimize power consumption or use larger battery packs
5. **Camera Integration**: Add OV7670 for visual confirmation of detections
6. **Cloud Logging**: Store all mission data to remote server for post-analysis
7. **Swarm Robotics**: Network multiple robots for faster coverage
8. **Advanced Path Planning**: Implement A* or RRT algorithms instead of fixed patterns

---

## 📚 References

1. Aerial and Ground Robot Collaboration for Autonomous Mapping in Search and Rescue Missions — *MDPI*
2. A Review of the Current State of Emergency Response Robotics — *IEEE/Springer*
3. Thermal Imaging and GPR in SAR Operations — *Research paper on victim detection*
4. Robotic Assistance in Search and Rescue Operations — *IEEE Xplore*
5. Adafruit MLX90614 Documentation: https://learn.adafruit.com/
6. ESP32 Official Documentation: https://docs.espressif.com/

---

## 📝 Project Structure

```
CSSR-Robot/
├── CSSR-code.ino              # Main firmware
├── README.md                  # This file
├── ccp_journal.pdf            # Full technical paper
├── hardware/
│   ├── circuit_diagram.png    # Wiring schematic
│   └── pcb_layout.pdf         # (Optional) PCB design
├── firmware/
│   ├── motor_control.h        # Motor utility functions
│   └── sensor_fusion.h        # Data processing algorithms
├── docs/
│   ├── calibration_guide.md   # Sensor calibration steps
│   └── troubleshooting.md     # Common issues & fixes
└── tests/
    ├── unit_tests.ino         # Individual sensor tests
    └── field_simulation.md     # Test procedures
```

---

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit changes (`git commit -m "Add feature"`)
4. Push to branch (`git push origin feature/improvement`)
5. Open a Pull Request

---

## 📄 License

This project is licensed under the **MIT License** — see LICENSE file for details.

---

## 📞 Support & Contact

For questions, issues, or feedback:

- **GitHub Issues**: Open an issue in the repository
- **Email**: harivenkatesh1006@gmail.com

