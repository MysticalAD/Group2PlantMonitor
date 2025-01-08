// **********************************
// * Group 2 Capstone Project
// * Team members: Yuejia, Daniel, Aditya, Jathin and Julian
// * Instructed by Jun Wen
// * Date: 2024-07-11
// **********************************
// * Project Description:
// * This code is for an ESP32-based system that monitors
// * the temperature, humidity, and soil moisture levels of a plant.
// * The system uses a DHT11 sensor to measure temperature and humidity,
// * and a moisture sensor to measure soil moisture levels.
// * The system triggers an alarm and sends an alert to AWS IoT Core when the temperature
// * exceeds 35 degrees Celsius or when the soil moisture level is less than 1000.
// **********************************
// * Import libraries
#include <DHT.h>               // Library for DHT sensor
#include <WiFi.h>              // Library for WiFi
#include <ESP32Ping.h>         // Library for ping functionality
#include <time.h>              // Library for NTP synchronization
#include "music.h"             // Library for music notes
#include <WiFiClientSecure.h>  // Include the WiFiClientSecure library
#include "SecureCredentials.h" // Include the secrets file
#include <PubSubClient.h>      // Include the MQTT client library
#include <ArduinoJson.h>


// ! Constants declaration for function initPWM() to control LEDs and buzzer
const int greenLedChannel = 0;  // PWM channel for Green LED
const int redLedChannel = 1;    // PWM channel for Red LED
const int buzzerChannel = 2;    // PWM channel for Buzzer
const int pwmFrequency = 5000;  // Frequency for PWM
const int pwmResolution = 8;    // 8-bit resolution (0-255)
const int buzzerDutyCycle = 64; // Duty cycle for the buzzer to lower the volume

// ! Constants declaration for function connectToWiFi() to access WiFi
constexpr const char *ssid = "Brown-Guest";        // WiFi SSID
constexpr const char *password = "";               // WiFi Password
constexpr unsigned long WIFI_RETRY_DELAY_MS = 500; // Delay between WiFi connection attempts
constexpr unsigned long WIFI_RETRY_WAIT_MS = 1000; // Wait time before next WiFi connection attempt

// ! Constants declaration for function pingHost()
constexpr const char *host = PING_HOST;            // Host to ping
constexpr unsigned long PING_RETRY_DELAY_MS = 500; // Delay between Ping attempts

// ! Constants declaration for function syncNTP()
constexpr const char *ntpServer = NTP_SERVER;     // NTP server
constexpr unsigned long NTP_SYNC_DELAY_MS = 1000; // Delay between NTP time sync attempts

// ! Constants declaration for function setup() and loop()
constexpr unsigned long LOOP_DELAY_MS = 10000;   // Delay for the main loop
constexpr unsigned long RESTART_DELAY_MS = 3000; // Delay before restarting the ESP32 in milliseconds
constexpr int maxRetries = 3;                    // Maximum number of retries for WiFi, Ping, and NTP

// ! Constants declaration for function readDHT() to read temperature and humidity data
DHT dht(DHT_PIN, DHT11);                      // Initialize DHT sensor for DHT11
constexpr float TEMP_HIGH_THRESHOLD_C = 20.0; // High temperature threshold in Celsius
float temperature_c;
float temperature_f;
float humidity_P;

// ! Constants declaration for function readMoisture() to read soil moisture data
constexpr int MOISTURE_LOW_THRESHOLD = 1000; // Low moisture threshold
int moistureData;

// ! Constants declaration for function buzzerAndBlinkAlarm() to trigger alarm
constexpr unsigned long ALARM_HIGH_FREQUENCY = 2000;    // High frequency for the alarm
constexpr unsigned long ALARM_LOW_FREQUENCY = 500;      // Low frequency for the alarm
constexpr unsigned long ALARM_TONE_DURATION_MS = 100;   // Duration of each alarm tone
constexpr unsigned long ALARM_TOTAL_DURATION_MS = 3000; // Total duration of the alarm

// ! Constants declaration for funciton connectAWS() and mqttPublishMessage() to connect to AWS IoT Core
WiFiClientSecure net;         // Create a WiFiClientSecure to handle the MQTT connection
PubSubClient mqttClient(net); // Create a PubSubClient to handle the MQTT connection
String deviceID;              // Device ID for the AWS IoT Core
String AWS_IOT_PUBLISH_TOPIC;
constexpr unsigned long MQTT_RECONNECT_DELAY_MS = 3000; // Delay between reconnect attempts

// Functions declaration
void setup();                                                                                      // Setup function declaration
void initPWM();                                                                                    // Function to configure PWM functionalities
void loop();                                                                                       // Loop function declaration
void readDHT();                                                                                    // Function to read and check temperature and humidity data from DHT sensor
void readMoisture(); 
void readSoilMoisture();                                                                              // Function to read soil moisture levels
float readDistance();                                                                               // Function to read distance using the ultrasonic sensor
bool connectToWiFi();                                                                              // Connects the ESP32 to the specified WiFi network
bool pingHost();                                                                                   // Pings the specified host to check network connectivity
bool syncNTP();                                                                                    // Synchronizes the ESP32's time with an NTP (Network Time Protocol) server
void buzzerAndBlinkAlarm();                                                                        // Function to trigger alarm
bool connectAWS();                                                                                 // Function to connect to AWS IoT Core
void mqttPublishMessage(float humidity, float temperature_c, float temperature_f, int moistureData, float distance); // Function to publish MQTT messages

// setup() function
void setup()
{
    Serial.begin(115200); // Initialize serial communication at 115200 baud rate

    dht.begin(); // Initialize DHT sensor

    initPWM(); // Initialize PWM functionalities

    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); // Set the ultrasonic trigger pin as output
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);  // 

    deviceID = String(ESP.getEfuseMac(), HEX); // Get the device ID
    AWS_IOT_PUBLISH_TOPIC = deviceID + "/pub"; // Set the MQTT topic to publish messages

    // Connect to WiFi
    if (!connectToWiFi())
    {
        Serial.println("Failed to connect to WiFi: " + String(ssid)); // Print error message to serial
        ledcWrite(redLedChannel, 255);                                // Turn on Red LED
        delay(RESTART_DELAY_MS);                                      // Wait before restarting
        ESP.restart();                                                // Reboot the MCU
    }

    // Ping Google's DNS server
    if (!pingHost())
    {
        Serial.println("Failed to ping host: " + String(host)); // Print error message to serial
        ledcWrite(redLedChannel, 255);                          // Turn on Red LED
        delay(RESTART_DELAY_MS);                                // Wait before restarting
        ESP.restart();                                          // Reboot the MCU
    }

    // Synchronize NTP time
    if (!syncNTP())
    {
        Serial.println("Failed to sync NTP: " + String(ntpServer)); // Print error message to serial
        ledcWrite(redLedChannel, 255);                              // Turn on Red LED
        delay(RESTART_DELAY_MS);                                    // Wait before restarting
        ESP.restart();                                              // Reboot the MCU
    }

    // Connect to AWS Cloud
    if (!connectAWS())
    {
        Serial.println("Failed to connect to AWS Cloud: " + String(AWS_IOT_MQTT_SERVER)); // Print error message to serial
        ledcWrite(redLedChannel, 255);                                                    // Turn on Red LED
        delay(RESTART_DELAY_MS);                                                          // Wait before restarting
        ESP.restart();                                                                    // Reboot the ESP32 if AWS connection fails
    }

    // Play music when NTP sync is successful
    playSuccessTune(buzzerChannel);
}

// loop() function
void loop()
{
    Serial.println("........ Loop iteration started.........."); // Print loop iteration message

    // Check if WiFi is connected
    if (WiFi.status() == WL_CONNECTED)
    {
        ledcWrite(greenLedChannel, 255);      // Set Green LED brightness to maximum
        ledcWrite(redLedChannel, 0);          // Turn off Red LED
        Serial.println("WiFi is connected."); // Print message to serial
    }
    else
    {
        Serial.println("WiFi connection lost. Attempting to reconnect..."); // Print error message to serial
        ledcWrite(greenLedChannel, 0);                                      // Turn off Green LED
        ledcWrite(redLedChannel, 255);                                      // Turn on Red LED
        int retryCount = 0;                                                 // Initialize retry count
        bool reconnected = false;                                           // Initialize reconnection status

        // Try to reconnect to WiFi
        while (retryCount < maxRetries && !reconnected)
        {
            retryCount++; // Increment retry count
            Serial.print("Retrying ");
            Serial.print(retryCount);
            Serial.print("/");
            Serial.println(maxRetries);
            if (connectToWiFi())
            {
                reconnected = true;              // Set reconnection status to true
                ledcWrite(greenLedChannel, 255); // Set Green LED brightness to maximum
                ledcWrite(redLedChannel, 0);     // Turn off Red LED
                Serial.println("Reconnected to WiFi successfully!");
            }
            else
            {
                delay(WIFI_RETRY_WAIT_MS); // Wait before next attempt
            }
        }

        if (!reconnected)
        {
            Serial.println("Failed to reconnect to WiFi after 3 attempts."); // Print error message to serial
            ledcWrite(redLedChannel, 255);                                   // Turn on Red LED
            delay(RESTART_DELAY_MS);                                         // Wait before restarting
            ESP.restart();                                                   // Reboot the MCU
        }
    }

    // Print Date and Time
    struct tm timeinfo;
    if (getLocalTime(&timeinfo))
    {
        char timeStr[64];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
        Serial.println(timeStr); // Display formatted time on OLED
    }
    else
    {
        Serial.println("Time: N/A"); // Display "Time: N/A" if time is not available
    }

    readDHT(); // Read and check DHT data
    readSoilMoisture();
    readMoisture(); // Read moisture data

    float distance = readDistance(); // Read distance data

    // Check conditions and take actions
    if (temperature_c > TEMP_HIGH_THRESHOLD_C || moistureData < MOISTURE_LOW_THRESHOLD)
    {
        buzzerAndBlinkAlarm();                                                              // Trigger alarm
        mqttPublishMessage(humidity_P, temperature_c, temperature_f, moistureData, distance); // Send alert to AWS
    }
    else
    {
        ledcWrite(redLedChannel, 0);     // Turn off Red LED
        ledcWrite(greenLedChannel, 255); // Turn on Green LED at full brightness
        ledcWrite(buzzerChannel, 0);     // Turn off buzzer
    }

    delay(LOOP_DELAY_MS); // Delay before next loop iteration
}

// Function to configure PWM functionalities
void initPWM()
{
    // Configure LED PWM functionalities
    ledcSetup(greenLedChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(GREEN_LED_PIN, greenLedChannel);

    ledcSetup(redLedChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(RED_LED_PIN, redLedChannel);

    // Configure Buzzer PWM functionalities
    ledcSetup(buzzerChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(BUZZER_PIN, buzzerChannel);
}

// Function to connect to WiFi
bool connectToWiFi()
{
    Serial.println("Connecting to WiFi: " + String(ssid));
    int attempt = 0; // Initialize attempt count
    while (attempt < maxRetries)
    {
        attempt++;                  // Increment attempt count
        WiFi.begin(ssid, password); // Start WiFi connection with given SSID and password
        int retries = 0;            // Initialize retries count
        while (WiFi.status() != WL_CONNECTED && retries < maxRetries)
        {                               // Check if WiFi is not connected
            delay(WIFI_RETRY_DELAY_MS); // Wait before next check
            Serial.print(".");
            retries++; // Increment retries count
        }
        if (WiFi.status() == WL_CONNECTED)
        {
            Serial.println(""); // Print new line for readability
            Serial.println("WiFi is connected successfully.");
            return true; // Return true if connected
        }
        else
        {
            Serial.print("WiFi connection attempt failed. Retrying ");
            Serial.print(attempt);
            Serial.print("/");
            Serial.println(maxRetries);
            WiFi.disconnect();         // Disconnect from WiFi
            delay(WIFI_RETRY_WAIT_MS); // Wait before next attempt
        }
    }
    return false; // If all attempts fail, return false
}

// Function to ping host
bool pingHost()
{
    Serial.print("Pinging host: " + String(host));
    Serial.println("...");
    int retries = 0;
    while (!Ping.ping(host) && retries < maxRetries)
    {
        retries++; // Increment retries count
        Serial.print("Ping failed, retrying ");
        Serial.print(retries);
        Serial.print("/");
        Serial.println(maxRetries);
    }
    if (Ping.ping(host))
    {
        Serial.println("Ping successful.");
        return true;
    }
    else
    {
        return false;
    }
}

// Function to synchronize NTP time
bool syncNTP()
{
    Serial.println("Synchronizing NTP: " + String(ntpServer));
    String ntpMessage = "Fetching date and time from NTP server: " + String(ntpServer);
    Serial.println(ntpMessage);
    struct timeval tv = {0};     // Clear system time
    settimeofday(&tv, NULL);     // Set system time to zero
    configTime(0, 0, ntpServer); // Configure time with NTP server
    struct tm timeinfo;          // Structure to hold time information
    int retries = 0;             // Initialize retries count
    while (!getLocalTime(&timeinfo) && retries < maxRetries)
    {
        retries++; // Increment retries count
        Serial.print("Waiting for NTP time sync, retrying ");
        Serial.print(retries);
        Serial.print("/");
        Serial.println(maxRetries);
        delay(NTP_SYNC_DELAY_MS); // Wait before next attempt
    }
    if (timeinfo.tm_year > (2020 - 1900))
    {                                                                       // Check if the year is valid
        char timeStr[64];                                                   // Buffer to hold formatted time string
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo); // Format time string
        Serial.println(timeStr);                                            // Print formatted time to serial
        return true;                                                        // Return true if NTP sync was successful
    }
    else
    {
        return false;
    }
}

bool connectAWS() // Function to connect to AWS IoT Core
{
    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_ROOT_CA);         // Set the AWS Root CA certificate
    net.setCertificate(AWS_CERT_CRT);   // Set the device certificate
    net.setPrivateKey(AWS_PRIVATE_KEY); // Set the private key
    // Set the AWS IoT endpoint and port
    mqttClient.setServer(AWS_IOT_MQTT_SERVER, AWS_IOT_MQTT_PORT);
    Serial.println("Connecting to AWS IoT Core...");
    int attempt = 0;
    while (!mqttClient.connect(deviceID.c_str()) && attempt < 3)
    {
        attempt++;
        Serial.print("Attempt ");
        Serial.print(attempt);
        Serial.println("/3 failed, retrying...");
        delay(MQTT_RECONNECT_DELAY_MS); // Delay before retrying
    }
    if (mqttClient.connected())
    {
        Serial.println("Connected to AWS Cloud successfully!");
        return true;
    }
    else
    {
        Serial.println("Failed to connect to AWS Cloud after 3 attempts.");
        return false;
    }
}

void readSoilMoisture()
{
    Serial.print("Soil Moisture:");
    Serial.println(digitalRead(SOIL_MOISTURE_PIN));
}

// Function to read, check, and send DHT data
void readDHT()
{
    humidity_P = dht.readHumidity();           // Read humidity
    temperature_c = dht.readTemperature();     // Read temperature in Celsius
    temperature_f = dht.readTemperature(true); // Read temperature in Fahrenheit

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity_P) || isnan(temperature_c) || isnan(temperature_f))
    {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }
    // Print to serial port
    Serial.print(F("Humidity: "));
    Serial.print(humidity_P);
    Serial.print(F("% "));
    Serial.print(F("Temperature: "));
    Serial.print(temperature_c);
    Serial.print((char)176); // ASCII code for degree symbol
    Serial.print(F("C "));
    Serial.print(temperature_f);
    Serial.print((char)176); // ASCII code for degree symbol
    Serial.println(F("F"));
}

// Function to trigger alarm
void buzzerAndBlinkAlarm()
{
    unsigned long startTime = millis();                    // Record the start time
    while (millis() - startTime < ALARM_TOTAL_DURATION_MS) // Continue the alarm during the specified duration
    {
        // Play high frequency and turn on Red LED
        ledcWriteTone(buzzerChannel, ALARM_HIGH_FREQUENCY);
        ledcWrite(buzzerChannel, buzzerDutyCycle); // Set the duty cycle for lower volume
        ledcWrite(redLedChannel, 255);             // Turn on Red LED
        delay(ALARM_TONE_DURATION_MS);
        // Play low frequency and turn off Red LED
        ledcWriteTone(buzzerChannel, ALARM_LOW_FREQUENCY);
        ledcWrite(buzzerChannel, buzzerDutyCycle); // Set the duty cycle for lower volume
        ledcWrite(redLedChannel, 0);               // Turn off Red LED
        delay(ALARM_TONE_DURATION_MS);
    }
    // Turn off the Buzzer
    ledcWriteTone(buzzerChannel, 0);
}

// Function to read soil moisture levels
void readMoisture()
{
    moistureData = analogRead(MOISTURE_SENSOR_PIN);

    Serial.print("Moisture: ");
    Serial.println(moistureData);
}

// Function to read distance using the ultrasonic sensor
// Function to read distance using the ultrasonic sensor
float readDistance()
{
    // Clear the trigger pin
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(5);

    // Set the trigger pin HIGH for 10 microseconds
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(20); // Increased pulse width to 20 microseconds
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    // Read the echo pin, and return the sound wave travel time in microseconds
    long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000); // Timeout after 30ms

    // Print duration for debugging
    Serial.print("Duration: ");
    Serial.print(duration);
    Serial.println(" microseconds");

    // Calculate the distance in centimeters
    float distance = duration * 0.034 / 2;

    // Print the distance to the Serial Monitor with 2 decimal places
    char distanceStr[10];
    dtostrf(distance, 6, 2, distanceStr);
    Serial.print("Distance: ");
    Serial.print(distanceStr);
    Serial.println(" cm");

    return distance;
}

// Function to publish MQTT messages
void mqttPublishMessage(float humidity, float temperature_c, float temperature_f, int moistureData, float distance)
{
    if (mqttClient.connected())
    {
        // Fetch the current time
        time_t unixTime = time(nullptr); // Get the current time as Unix time
        // Check if time is valid
        if (unixTime == -1)
        {
            Serial.println("Failed to obtain time");
            return;
        }
        // Create a JSON document
        StaticJsonDocument<512> doc;
        String action = "";
        if (temperature_c > TEMP_HIGH_THRESHOLD_C)
        {
            action = "Move plant inside room.";
        }
        if (moistureData < MOISTURE_LOW_THRESHOLD)
        {
            action = "Irrigate the plant.";
        }
        // Populate document
        doc["timeStamp"] = unixTime;                      // Add timestamp to JSON document
        doc["deviceModel"] = "ESP32";                     // Add device model to JSON document
        doc["owner"] = "Yuejia";                          // Add owner to JSON document
        doc["group"] = "Group_2";                         // Add group to JSON document
        doc["deviceID"] = String(ESP.getEfuseMac(), HEX); // Add device ID to JSON document
        JsonObject data = doc.createNestedObject("data"); // Create a nested object for data
        data["temp_c"] = round(temperature_c);            // Add temperature in Celsius to JSON document
        data["temp_f"] = round(temperature_f);            // Add temperature in Fahrenheit to JSON document
        data["humidity"] = humidity;                      // Add humidity to JSON document
        data["moistureValue"] = moistureData;             // Add moisture value to JSON document
        data["distance"] = round(distance); // Add formatted distance to JSON document
        data["lat"] = "41.8264";                     // Add latitude to JSON document
        data["lon"] = "-71.3977";                   // Add longitude to JSON document

        String jsonString;                    // Create a string to hold the JSON data
        serializeJson(doc, jsonString);       // Serialize the JSON document to a string
        Serial.print("Publishing message: "); // Print the message
        Serial.println(jsonString);           // Print the JSON data

        // Determine buffer size
        size_t jsonSize = measureJson(doc) + 1; // +1 for null terminator
        Serial.print("Calculated JSON buffer size: ");
        Serial.println(jsonSize); // Print the buffer size
        if (!mqttClient.publish(AWS_IOT_PUBLISH_TOPIC.c_str(), jsonString.c_str()))
        {
            Serial.println("Publish failed"); // Print publish failure message
        }
        else
        {
            Serial.println("Publish succeeded"); // Print publish success message
        }
        doc.clear();     // Clear the JSON document
        jsonString = ""; // Clear the JSON string
    }
}
