Sure, here's a copy-and-paste version formatted for GitHub:

---

# ICM42605-Esp32 Library

## Overview
The **ICM42605-Esp32** library is developed by GIBIC TechHub to interface the ICM42605 sensor with ESP32 microcontrollers. This library provides an easy-to-use interface for configuring and reading data from the ICM42605, a high-performance 6-axis MEMS sensor that combines a 3-axis gyroscope and a 3-axis accelerometer.

## Features
- Simple and intuitive API for interacting with the ICM42605 sensor.
- Support for configuring gyroscope and accelerometer settings.
- Functions to read raw and processed sensor data.
- Integration with ESP32's I2C communication.

## Installation
1. **Clone the Repository**
   ```sh
   git clone https://github.com/GIBICTechHub/ICM42605-Esp32.git
   ```

2. **Copy to Arduino Libraries Folder**
   Copy the `ICM42605-Esp32` folder to your Arduino libraries directory, typically found at `~/Documents/Arduino/libraries/`.

## Usage
1. **Include the Library**
   ```cpp
   #include <ICM42605_Esp32.h>
   ```

2. **Initialize the Sensor**
   ```cpp
   ICM42605 sensor;
   
   void setup() {
     Serial.begin(115200);
     Wire.begin();
     if (!sensor.begin()) {
       Serial.println("Failed to initialize ICM42605!");
       while (1);
     }
     Serial.println("ICM42605 initialized successfully!");
   }
   ```

3. **Reading Sensor Data**
   ```cpp
   void loop() {
     sensor.update();
     float ax = sensor.getAccelX();
     float ay = sensor.getAccelY();
     float az = sensor.getAccelZ();
     float gx = sensor.getGyroX();
     float gy = sensor.getGyroY();
     float gz = sensor.getGyroZ();
     
     Serial.print("Accel: ");
     Serial.print(ax); Serial.print(", ");
     Serial.print(ay); Serial.print(", ");
     Serial.println(az);
     
     Serial.print("Gyro: ");
     Serial.print(gx); Serial.print(", ");
     Serial.print(gy); Serial.print(", ");
     Serial.println(gz);
     
     delay(500);
   }
   ```

## API Reference

### Initialization
- **`bool begin();`**
  Initializes the ICM42605 sensor. Returns `true` if successful, `false` otherwise.

### Data Reading
- **`void update();`**
  Reads the latest sensor data and updates internal variables.

- **`float getAccelX();`** 
  Returns the X-axis acceleration.

- **`float getAccelY();`**
  Returns the Y-axis acceleration.

- **`float getAccelZ();`**
  Returns the Z-axis acceleration.

- **`float getGyroX();`**
  Returns the X-axis gyroscope data.

- **`float getGyroY();`**
  Returns the Y-axis gyroscope data.

- **`float getGyroZ();`**
  Returns the Z-axis gyroscope data.

### Configuration
- **`void setAccelRange(uint8_t range);`**
  Sets the accelerometer range. Possible values are `2G`, `4G`, `8G`, `16G`.

- **`void setGyroRange(uint16_t range);`**
  Sets the gyroscope range. Possible values are `125DPS`, `250DPS`, `500DPS`, `1000DPS`, `2000DPS`.

## Contributing
We welcome contributions to enhance this library. Please fork the repository, make your changes, and submit a pull request.

## License
This project is licensed under the MIT License. See the `LICENSE` file for more details.

---

Feel free to copy and paste this into your GitHub repository's README file.
