/*##################################################################################

EXAMPLE USING 'delayMicroseconds' to test and improve buzzer operation

####################################################################################*/

int outPin = 8;               // digital pin 8

void setup() {
  pinMode(outPin, OUTPUT);    // sets the digital pin as output
}

void loop() {
  digitalWrite(outPin, HIGH); // sets the pin on
  delayMicroseconds(50);      // pauses for 50 microseconds
  digitalWrite(outPin, LOW);  // sets the pin off
  delayMicroseconds(50);      // pauses for 50 microseconds
}

/*##################################################################################

EXAMPLE Talking directly with the BMI270 on I2C buss

####################################################################################*/

#include <Arduino.h>
#include <Wire.h>

// Define the SCL and SDA pins on the ESP32-S2
#define SCL_PIN 6
#define SDA_PIN 7

// Define the I2C address of the BMI270 MPU module
#define BMI270_I2C_ADDRESS 0x68

void setup() {
  // Set the SCL and SDA pins as I2C pins
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, OUTPUT);

  // Initialize the I2C bus
  Wire.begin(SCL_PIN, SDA_PIN, 400000);

  // Print a message to the serial monitor
  Serial.println("BMI270 MPU Example");
}

void loop() {
  // Read the accelerometer data from the BMI270 MPU module
  int16_t accelX, accelY, accelZ;
  Wire.requestFrom(BMI270_I2C_ADDRESS, 6);
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();

  // Print the accelerometer data to the serial monitor
  Serial.print("Accelerometer data: ");
  Serial.print(accelX);
  Serial.print(", ");
  Serial.print(accelY);
  Serial.print(", ");
  Serial.println(accelZ);
  // Delay for 1 second
  delay(1000);

}

/* ######################################################################################################################
To use this program, you will need to connect the SCL and SDA pins of the BMI270 MPU module to the GPIO6 and GPIO7 pins of the ESP32-S2 module, respectively.
You will also need to connect the VCC and GND pins of the BMI270 MPU module to the 3.3V and GND pins of the ESP32-S2 module.
Once you have connected the two modules, you can upload the program to the ESP32-S2 module using the Arduino IDE. Once the program is uploaded,
 you can open the serial monitor to view the accelerometer data from the BMI270 MPU module.
 #######################################################################################################################*/
 
 /*##################################################################################

EXAMPLE SETTING GPIO9 TO A PWM OUTPUT AT 2700HZ 50% DUTY CYCLE

####################################################################################*/

#include <Arduino.h>
#include <ledc.h>

// Define the PWM channel for GPIO9
#define PWM_CHANNEL 1

// Define the PWM frequency in Hz
#define PWM_FREQUENCY 2700

// Define the PWM duty cycle in percent
#define PWM_DUTY_CYCLE 50

void setup() {
  // Set GPIO9 as a PWM output
  pinMode(9, OUTPUT);
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, 8);
  ledcAttachPin(9, PWM_CHANNEL);

  // Set the PWM duty cycle
  ledcWrite(PWM_CHANNEL, PWM_DUTY_CYCLE);
  
}

void loop() {
  // Nothing to do in the loop
}


/*##################################################################################

EXAMPLE CODE FOR SCANNING AN I2C BUSS TO FIND OUT WHAT ADDRESSES ARE CONNECTED
FOR EACH DEVICE 

####################################################################################*/

/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <Wire.h>
 
void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
}
 
void loop() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);          
}