/*#######################################################################################################

The following Arduino program for an ESP32-S2 turns on an LED attached to GPIO10 for 10 seconds when a
button is pushed connected to GPIO11, without using the delay() function:

#########################################################################################################*/

#include <Arduino.h>

const int buttonPin = 11;
const int ledPin = 10;

unsigned long buttonPressedTime = 0;
unsigned long ledOnTime = 0;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  ledOff();
}

void loop() {
  if (digitalRead(buttonPin) == LOW) {
    buttonPressedTime = millis();
  }

  if (millis() >= buttonPressedTime + 10000) {
    ledOff();
  }

  if (ledOnTime > 0 && millis() >= ledOnTime) {
    ledOff();
  }
}

void ledOn() {
  digitalWrite(ledPin, HIGH);
  ledOnTime = millis() + 10000;
}

void ledOff() {
  digitalWrite(ledPin, LOW);
  ledOnTime = 0;
}


#include <Arduino.h>

const int buttonPin = 11;
const int ledPin = 10;

unsigned long buttonPressedTime = 0;
unsigned long ledOnTime = 0;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  ledOff();
}

void loop() {
  if (digitalRead(buttonPin) == LOW) {
    buttonPressedTime = millis();
  }

  if (millis() >= buttonPressedTime + 10000) {
    ledOff();
  }

  if (ledOnTime > 0 && millis() >= ledOnTime) {
    ledOff();
  }
}

void ledOn() {
  digitalWrite(ledPin, HIGH);
  ledOnTime = millis() + 10000;
}

void ledOff() {
  digitalWrite(ledPin, LOW);
  ledOnTime = 0;
}


/*#######################################################################################################

The following Arduino program for an ESP32-S2 turns on an LED attached to GPIO10 for 10 seconds, 
then turns off the LED and goes into Deep Sleep until an interrupt signal goes from low to high on GPIO11,
without using the delay() function:

#########################################################################################################*/

#include <Arduino.h>

const int ledPin = 10;
const int buttonPin = 11;

unsigned long ledOnTime = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), wakeUpFromDeepSleep, RISING);
  ledOff();
}

void loop() {
  if (millis() >= ledOnTime) {
    ledOff();
    esp_deep_sleep_start();
  }
}

void ledOn() {
  digitalWrite(ledPin, HIGH);
  ledOnTime = millis() + 10000;
}

void ledOff() {
  digitalWrite(ledPin, LOW);
  ledOnTime = 0;
}

void wakeUpFromDeepSleep() {
  ledOn();
}

/*#######################################################################################################

Write an Arduino program for an ESP32-S2 that measures an analog voltage between 200mV and 900mV on GPIO10,
when a button on GPIO11 is pushed. The program then sends the voltage value in Volts to the Serial monitor.

#########################################################################################################*/

#include <Arduino.h>

const int analogPin = 10;
const int buttonPin = 11;

float voltage = 0.0;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  if (digitalRead(buttonPin) == LOW) {
    // Button is pressed, read analog voltage
    voltage = analogRead(analogPin) * 3.3 / 4096.0;

    // Convert voltage to Volts
    voltage /= 1000.0;

    // Print voltage to Serial monitor
    Serial.println(voltage);
  }
}

/*#######################################################################################################

The following Arduino program for an ESP32-S2 measures an analog voltage between 200mV and 900mV on 
GPIO10, GPIO11, and GPIO12, when a button on GPIO15 is pushed, and then stores the values
in a `Values.json` file as `Voltage_1`, `Voltage_2`, and `Voltage_3`:

#######################################################################################################*/

/*########################################################################################################

This program works by first setting the button and analog pins as inputs. It then initializes the
 Serial monitor at a baud rate of 115200 and initializes SPIFFS.

In the `loop()` function, the program checks the state of the button pin. If the button is pressed,
the program reads the analog voltages on GPIO10, GPIO11, and GPIO12 using the `analogRead()` function. 
The program then converts the analog voltages to Volts by dividing by 1000.0.

Next, the program creates a JSON object with the voltage values. The program then serializes the 
JSON object to a string and opens the `Values.json` file for writing. If the file cannot be opened,
the program prints an error message to the Serial monitor and returns.

If the file can be opened, the program writes the JSON string to the file and closes the file.
 Finally, the program prints a success message to the Serial monitor.

#########################################################################################################**/



#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

const int analogPin1 = 10;
const int analogPin2 = 11;
const int analogPin3 = 12;
const int buttonPin = 15;

float voltages[3];

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS initialization failed!");
    return;
  }
}

void loop() {
  if (digitalRead(buttonPin) == LOW) {
    // Button is pressed, read analog voltages
    voltages[0] = analogRead(analogPin1) * 3.3 / 4096.0;
    voltages[1] = analogRead(analogPin2) * 3.3 / 4096.0;
    voltages[2] = analogRead(analogPin3) * 3.3 / 4096.0;

    // Convert voltages to Volts
    for (int i = 0; i < 3; i++) {
      voltages[i] /= 1000.0;
    }

    // Create JSON object
    StaticJsonDocument<256> doc;
    doc["Voltage_1"] = voltages[0];
    doc["Voltage_2"] = voltages[1];
    doc["Voltage_3"] = voltages[2];

    // Serialize JSON object to string
    char json[256];
    serializeJson(doc, json);

    // Open Values.json file for writing
    File file = SPIFFS.open("Values.json", FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open Values.json file for writing!");
      return;
    }

    // Write JSON string to file
    file.print(json);

    // Close file
    file.close();

    // Print success message to Serial monitor
    Serial.println("Voltages stored successfully!");
  }
}


/*########################################################################################################

This program works by first setting the button and analog pins as inputs. It then initializes the 
Serial monitor at a baud rate of 115200 and initializes SPIFFS.

In the loop() function, the program checks the state of the button pin. If the button is pressed, 
the program reads the analog voltages on GPIO10, GPIO11, and GPIO12 using the analogRead() function. 
The program then converts the analog voltages to Volts by dividing by 1000.0.

Next, the program creates a JSON object with the voltage values. The program then serializes 
the JSON object to a string and opens the Values.json file for writing. If the file cannot be opened, 
the program prints an error message to the Serial monitor and returns.

If the file can be opened, the program writes the JSON string to the file and closes the file. Finally,
the program prints a success message to the Serial monitor.

#########################################################################################################**/

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

const int analogPin1 = 10;
const int analogPin2 = 11;
const int analogPin3 = 12;
const int buttonPin = 15;

float voltages[3];

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS initialization failed!");
    return;
  }
}

void loop() {
  if (digitalRead(buttonPin) == LOW) {
    // Button is pressed, read analog voltages
    voltages[0] = analogRead(analogPin1) * 3.3 / 4096.0;
    voltages[1] = analogRead(analogPin2) * 3.3 / 4096.0;
    voltages[2] = analogRead(analogPin3) * 3.3 / 4096.0;

    // Convert voltages to Volts
    for (int i = 0; i < 3; i++) {
      voltages[i] /= 1000.0;
    }

    // Create JSON object
    StaticJsonDocument<256> doc;
    doc["Voltage_1"] = voltages[0];
    doc["Voltage_2"] = voltages[1];
    doc["Voltage_3"] = voltages[2];

    // Serialize JSON object to string
    char json[256];
    serializeJson(doc, json);

    // Open Values.json file for writing
    File file = SPIFFS.open("Values.json", FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open Values.json file for writing!");
      return;
    }

    // Write JSON string to file
    file.print(json);

    // Close file
    file.close();

    // Print success message to Serial monitor
    Serial.println("Voltages stored successfully!");
  }
}

/*########################################################################################################



