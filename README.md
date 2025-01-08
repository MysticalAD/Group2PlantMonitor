# Group2PlantMonitor
This project entails a simple plant monitoring system which can be used to tend to the plant when it is facing water deficiency, humid atmosphere, excessive rain or low soil moisture. It was a part of a capstone project I did with fellow coders at Brown University at a summer course "Apply ChatGPT to accelerate your innovation".

Here are some key features of this project:
*   **Device:** The core of the system is the ESP32-C3-DevKitC-02, which is an entry-level development board with Wi-Fi and Bluetooth LE capabilities.
*   **Sensors:** The system utilizes multiple sensors to gather environmental data. These include:
    *   An **ultrasonic sensor** that can be used for obstacle detection or object detection.
    *   A **DHT11 sensor** for measuring temperature and humidity.
    *   A **rain/water drop detection sensor**.
    *   A **soil moisture detection sensor**.
*   **Functionality:** The system operates by connecting to Wi-Fi, syncing to NTP, and then to AWS. The sensors measure the temperature, humidity, and soil moisture. If certain environmental criteria are met (for example, temperature is above 35 degrees, humidity is above 65, or it is raining) a warning message will be sent and a buzzer will sound, and the red LED will light. Also an email will be sent through AWS.
*   **Coding Environment**: The project was developed using the Arduino framework and Platformio IDE.
*   **Real-time Monitoring**: The system provides real-time data monitoring through a time series chart which visualises temperature and humidity.

The system’s aim is to provide **early warnings of conditions that could harm plants**. For example, monitoring temperature and humidity can indicate when a plant needs watering, and monitoring for rain can alert users to potentially damaging storms. This system also enables the monitoring of soil moisture to help ensure optimal growing conditions and efficient water usage.

This project is designed to be instructional with simple, readable code. The project's example code includes libraries, declarations, functions, setup and loop examples, with clear comments to support learning.
