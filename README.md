# ESP32 Automotive Cruise Control System

This project is an ESP32-based cruise control system for vehicles. It integrates multiple sensors and components, including PID control, OBD-II communication, Bluetooth, Inside/Outside temperature monitoring, and a TFT display. The main goal is to provide a cruise control solution using an ESP32 microcontroller.

## Features
- **PID Control**: Maintains the target speed using PID algorithms.
- **Bluetooth Integration**: Connects via Bluetooth for OBD-II data reading and control.
- **OBD-II Data Reading**: Retrieves speed and engine data using ELM327.
- **Temperature Monitoring**: Reads ambient and cabin temperatures using DHT11 and DallasTemperature sensors.
- **TFT Display**: Real-time display of speed, target speed, and other sensor data.

## Demo
![demo](https://successdt.github.io/images/IMG_20210926_160237.jpg)

[https://www.youtube.com/watch?v=7pOyFUubQzo](https://www.youtube.com/watch?v=7pOyFUubQzo)


## Getting Started

### Prerequisites
- **Hardware**:
  - ESP32 microcontroller
  - DHT11 temperature sensor
  - DS18B20 temperature sensor
  - TFT display module
  - ELM327 Bluetooth OBD-II adapter
  - Buttons for control (SET, RES, ON)
  - PCB Schematic diagram
  ![Schematic](https://successdt.github.io/images/Schematic_Cruise-Control-v2_2024-11-15.png)
- **Software**:
  - Arduino IDE
  - ESP32 Board package for Arduino IDE
  - Libraries:
    - [DHT](https://github.com/adafruit/DHT-sensor-library)
    - [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library)
    - [BluetoothSerial](https://github.com/espressif/arduino-esp32)
    - [ELMduino](https://github.com/PowerBroker2/ELMduino)
    - [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI)


### Installation
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/successdt/esp32_cruise_control.git
   cd esp32_cruise_control
   ```
2. **Install PlatformIO extension to your Visual Studio Code**
3. **Open the Project**
4. **Build and upload to your ESP32 board**

### Contributing
Contributions are welcome! Here's how you can help:

1. **Fork the Project:**

Click on the Fork button at the top right corner of the GitHub page.
Create Your Feature Branch:


```bash
Copy code
git checkout -b feature/NewFeature
```
2. **Commit Your Changes:**

```bash
Copy code
git commit -m "Add NewFeature"
```
3. **Push to the Branch:**

```bash
Copy code
git push origin feature/NewFeature
```

4. **Open a Pull Request.**

### Issues
If you encounter any problems or have questions, please open an issue.

### License
This project is licensed under the MIT License - see the LICENSE file for details.

Acknowledgements

### Contact
Feel free to reach out via GitHub for any inquiries or collaboration.
