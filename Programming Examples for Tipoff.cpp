
/*#############################################################################################

Sending a JSON file to a server

I can provide you with a basic example code to send a JSON file from an ESP32 to a website. Here's an example using the Arduino framework and the ESP32 WiFi library:
################################################################################################*/


#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";
const char* serverUrl = "http://example.com/upload";  // Replace with your server URL

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");
}

void loop() {
  // Create a JSON payload
  const char* jsonPayload = "{\"data\": \"Hello, World!\"}";
  
  // Send the JSON payload to the server
  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "application/json");
  
  int httpResponseCode = http.POST(jsonPayload);
  
  if (httpResponseCode > 0) {
    Serial.printf("HTTP Response code: %d\n", httpResponseCode);
    String response = http.getString();
    Serial.println(response);
  } else {
    Serial.printf("Error code: %d\n", httpResponseCode);
  }
  
  http.end();
  
  delay(5000);  // Wait for 5 seconds before sending the next request
}
```
/*
In this example, you need to replace `"YourWiFiSSID"` and `"YourWiFiPassword"` with your actual Wi-Fi credentials. Also, replace `"http://example.com/upload"` with the URL of your server that will receive the JSON file.

To compile and upload this code to your ESP32 board, you will need the Arduino IDE with the ESP32 board support package installed. You can find more information about setting up the Arduino IDE for ESP32 development in the official documentation: 
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)

For more details on the HTTPClient library, you can refer to the official documentation:
- [HTTPClient - Arduino Reference](https://www.arduino.cc/en/Reference/HTTPClient)

Remember to adapt the code to your specific requirements, such as the JSON payload structure and the server endpoint.
*/

/* #############################################################################################

Communication with the BMI270 MPU

################################################################################################*/

#include <Wire.h>
#include <BMI270.h>

// Define the MPU object
BMI270 mpu;

// Define GPIO pin
const int gpioPin = 5;

// Define variables for MPU data
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize I2C communication
  Wire.begin();
  
  // Initialize MPU
  if (!mpu.begin()) {
    Serial.println("MPU initialization failed!");
    while (1);
  }
  
  // Set GPIO pin as output
  pinMode(gpioPin, OUTPUT);
}

void loop() {
  // Read MPU data
  mpu.readAccel(&accelX, &accelY, &accelZ);
  mpu.readGyro(&gyroX, &gyroY, &gyroZ);
  
  // Print MPU data to serial monitor
  Serial.print("Accel (X,Y,Z): ");
  Serial.print(accelX);
  Serial.print(", ");
  Serial.print(accelY);
  Serial.print(", ");
  Serial.println(accelZ);
  
  Serial.print("Gyro (X,Y,Z): ");
  Serial.print(gyroX);
  Serial.print(", ");
  Serial.print(gyroY);
  Serial.print(", ");
  Serial.println(gyroZ);
  
  // Check if the device is moved
  if (abs(accelX) > 1 || abs(accelY) > 1 || abs(accelZ) > 1) {
    // Turn on GPIO5
    digitalWrite(gpioPin, HIGH);
  } else {
    // Turn off GPIO5
    digitalWrite(gpioPin, LOW);
  }
  
  // Delay for 100 milliseconds
  delay(100);
}

/* #############################################################################################

Communication with the BMI270 MPU attaching to an interupt to wake up

################################################################################################*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMI270.h>

#define INTERRUPT_PIN 4  // GPIO pin connected to interrupt pin of BMI270

Adafruit_BMI270 bmi;

void setup() {
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), wakeUp, RISING);
  
  // Initialize the BMI270 accelerometer
  if (!bmi.begin()) {
    Serial.println("Failed to initialize BMI270!");
    while (1);
  }
  
  // Configure the BMI270 interrupt
  bmi.enableInterrupt(BMI270_INT1, BMI270_INT1_PIN, RISING);
  
  // Set sleep mode to enable waking up from external interrupt
  esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, HIGH);
}

void loop() {
  // Perform normal operations here
  
  // Put the ESP32 into deep sleep mode
  esp_deep_sleep_start();
}

void wakeUp() {
  // Interrupt handler function
  // Add any necessary code to handle the interrupt here
  
  // Code to turn on the output would also go here
}

/* #############################################################################################

Turn on GPIO6 for a specified number of minutes and then turn off the OUTPUT and go into deep sleep.

################################################################################################*/

#include <WiFi.h>

const int ledPin = 6;  // GPIO6 pin

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  
  // Connect to WiFi
  WiFi.begin("your_SSID", "your_password");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  // Wait for the number of minutes entered via serial monitor
  int minutes = 0;
  while (Serial.available() == 0) {
    delay(100);
  }
  minutes = Serial.parseInt();
  
  // Turn on the LED
  digitalWrite(ledPin, HIGH);
  Serial.println("LED turned on");
  
  // Wait for the specified number of minutes
  unsigned long endTime = millis() + (minutes * 60 * 1000);
  while (millis() < endTime) {
    // Do any other tasks if needed
  }
  
  // Turn off the LED
  digitalWrite(ledPin, LOW);
  Serial.println("LED turned off");
  
  // Go into deep sleep
  esp_deep_sleep_start();
}


/* #############################################################################################

Recieving and parsing a JSON file on the ESP32.

################################################################################################*/


#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";
const char* serverUrl = "http://your_server_url/file.json";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");
  
  HTTPClient http;
  http.begin(serverUrl);
  
  int httpResponseCode = http.GET();
  
  if (httpResponseCode == 200) {
    String payload = http.getString();
    
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);
    
    int value1 = doc["Value1"];
    int value2 = doc["Value2"];
    int value3 = doc["Value3"];
    
    Serial.print("Value1: ");
    Serial.println(value1);
    
    Serial.print("Value2: ");
    Serial.println(value2);
    
    Serial.print("Value3: ");
    Serial.println(value3);
  }
  
  http.end();
}

void loop() {
  // Your code here
}

/* #############################################################################################
Make sure to replace your_SSID, your_PASSWORD, and your_server_url/file.json with the appropriate values for your network and server.

To compile and upload this code to your ESP32, you'll need to have the ESP32 board package installed in your Arduino IDE.
You can find more information on how to set up the ESP32 board package in the official documentation: https://github.com/espressif/arduino-esp32

Additionally, you'll need to install the ArduinoJson library for parsing JSON.
You can install it through the Arduino Library Manager or manually download it from the ArduinoJson GitHub repository: https://github.com/bblanchon/ArduinoJson

################################################################################################*/

/* #############################################################################################
Make sure to replace your_SSID, your_PASSWORD, and your_server_url/file.json with the appropriate values for your network and server.

To compile and upload this code to your ESP32, you'll need to have the ESP32 board package installed in your Arduino IDE.
You can find more information on how to set up the ESP32 board package in the official documentation: https://github.com/espressif/arduino-esp32

Additionally, you'll need to install the ArduinoJson library for parsing JSON.
You can install it through the Arduino Library Manager or manually download it from the ArduinoJson GitHub repository: https://github.com/bblanchon/ArduinoJson

WRITING THE JSON DATA TO A BLUETOOTH SERIAL WINDOW.

################################################################################################*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <BluetoothSerial.h>

const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";
const char* serverUrl = "http://your-server.com/json-file.json";

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT"); // Bluetooth device name

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  SerialBT.begin("ESP32_BT");
  Serial.println("Bluetooth serial started");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    Serial.print("Connecting to server: ");
    Serial.println(serverUrl);

    http.begin(serverUrl);
    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) {
      String payload = http.getString();

      DynamicJsonDocument doc(1024);
      deserializeJson(doc, payload);

      int value1 = doc["Value1"];
      int value2 = doc["Value2"];
      int value3 = doc["Value3"];

      Serial.print("Value1: ");
      Serial.println(value1);
      Serial.print("Value2: ");
      Serial.println(value2);
      Serial.print("Value3: ");
      Serial.println(value3);

      SerialBT.print("Value1: ");
      SerialBT.println(value1);
      SerialBT.print("Value2: ");
      SerialBT.println(value2);
      SerialBT.print("Value3: ");
      SerialBT.println(value3);
    }
    else {
      Serial.print("HTTP request failed with error code: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  }

  delay(5000); // Wait for 5 seconds before making the next request
}
/* #############################################################################################


To compile and upload this code to your ESP32, you'll need to have the ESP32 board package installed in your Arduino IDE.
You can find more information on how to set up the ESP32 board package in the official documentation: https://github.com/espressif/arduino-esp32

Additionally, you'll need to install the ArduinoJson library for parsing JSON.
You can install it through the Arduino Library Manager or manually download it from the ArduinoJson GitHub repository: https://github.com/bblanchon/ArduinoJson

WRITING THE JSON DATA TO LOCAL JSON FILE ON THE ESP32.

################################################################################################*/


#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* serverUrl = "http://example.com/jsonfile.json";

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }

  // Download JSON file
  HTTPClient http;
  http.begin(serverUrl);
  int httpResponseCode = http.GET();

  if (httpResponseCode == 200) {
    String payload = http.getString();

    // Parse JSON
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
      return;
    }

    // Extract values
    int value1 = doc["Value1"];
    int value2 = doc["Value2"];
    int value3 = doc["Value3"];

    Serial.print("Value1: ");
    Serial.println(value1);
    Serial.print("Value2: ");
    Serial.println(value2);
    Serial.print("Value3: ");
    Serial.println(value3);

    // Store values in a JSON file
    File file = SPIFFS.open("/Local.json", FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file for writing");
      return;
    }

    doc["Value1"] = value1;
    doc["Value2"] = value2;
    doc["Value3"] = value3;

    if (serializeJson(doc, file) == 0) {
      Serial.println("Failed to write to file");
    }

    file.close();
  } else {
    Serial.print("HTTP request failed with error code: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void loop() {
  //


/* #############################################################################################

GETTING THE TIME FROM AN NPT SERVER

################################################################################################*/

#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  timeClient.begin();
  timeClient.setTimeOffset(3600); // Set your timezone offset in seconds
}

void loop() {
  timeClient.update();
  
  Serial.print("Current time: ");
  Serial.println(timeClient.getFormattedTime());
  
  delay(1000); // Wait for a second
}


/* #############################################################################################

Sending MPU data to a Google Spreadsheet

################################################################################################*/

Certainly! Here's an example code that demonstrates how to interface an ESP32-S2 with a BMI270 MPU to obtain motion information and send the data to a Google Spreadsheet:

```cpp
#include <Wire.h>
#include <BMI270.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Replace with your network credentials
const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

// Replace with your Google Spreadsheet URL
const char* spreadsheetURL = "https://docs.google.com/spreadsheets/d/your_spreadsheet_ID/edit#gid=0";

// Create an instance of the BMI270 library
BMI270 imu;

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize the BMI270 MPU
  Wire.begin();
  imu.initialize();

  // Enable motion detection
  imu.enableMotionDetection();
}

void loop() {
  // Read motion information from the BMI270
  imu.readMotion();

  // Get the accelerometer and gyroscope data
  float accelX = imu.getAccelX();
  float accelY = imu.getAccelY();
  float accelZ = imu.getAccelZ();
  float gyroX = imu.getGyroX();
  float gyroY = imu.getGyroY();
  float gyroZ = imu.getGyroZ();

  // Create the data payload to be sent to the Google Spreadsheet
  String payload = "accelX=" + String(accelX) +
                   "&accelY=" + String(accelY) +
                   "&accelZ=" + String(accelZ) +
                   "&gyroX=" + String(gyroX) +
                   "&gyroY=" + String(gyroY) +
                   "&gyroZ=" + String(gyroZ);

  // Send the data to the Google Spreadsheet
  HTTPClient http;
  http.begin(spreadsheetURL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int httpResponseCode = http.POST(payload);
  if (httpResponseCode == 200) {
    Serial.println("Data sent to the Google Spreadsheet successfully");
  } else {
    Serial.println("Error sending data to the Google Spreadsheet");
  }


/* #############################################################################################

Sending MPU data to a Google Spreadsheet

################################################################################################*/