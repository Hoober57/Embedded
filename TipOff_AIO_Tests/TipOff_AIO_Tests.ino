
//#include <Arduino.h>

const int analogPin1 = 1;  // GPIO1
const int outputPin16 = 16;  // GPIO15

const int analogPin3 = 3;  // GPIO3
const int outputPin15 = 15;  // GPIO15


void setup() {

  pinMode(outputPin15, OUTPUT);
  pinMode(outputPin16, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  
  Serial.println("##############################################################################");
  digitalWrite(outputPin15, HIGH);  // Turn on GPIO15
  Serial.println("Turning on GPIO15");
  digitalWrite(outputPin16, HIGH);  // Turn on GPIO15
  Serial.println("Turning on GPIO16");
  delay(2000);
  
  float voltage1 = analogRead(analogPin1) * 3.3 / 4095;  // Convert ADC value to voltage (3.3V reference)
  float voltage3 = analogRead(analogPin3) * 3.3 / 4095;
  
  Serial.print("Voltage on (BATT) GPIO1: ");
  Serial.print(voltage1);
  Serial.print("Voltage on (USB) GPIO3: ");
  Serial.print(voltage3);
  Serial.println("V");
  Serial.println("##############################################################################");

  digitalWrite(outputPin15, LOW);  // Turn on GPIO15
  Serial.println("Turning off GPIO15");
  digitalWrite(outputPin16, LOW);  // Turn on GPIO15
  Serial.println("Turning off GPIO16");
  delay(2000);

  voltage1 = analogRead(analogPin1) * 3.3 / 4095;  // Convert ADC value to voltage (3.3V reference)
  voltage3 = analogRead(analogPin3) * 3.3 / 4095;

  Serial.print("Voltage on (BATT) GPIO1: ");
  Serial.print(voltage1);
  Serial.print("V\tVoltage on (USB) GPIO3: ");
  Serial.print(voltage3);
  Serial.println("V");
  Serial.println("##############################################################################");
  delay(2000);

/*
  Serial.print("Voltage on GPIO1: ");
  Serial.print(voltage1);
  Serial.print("V\tVoltage on GPIO3: ");
  Serial.print(voltage3);
  Serial.println("V");

  digitalWrite(outputPin, LOW);   // Turn off GPIO15
  Serial.println("Turning off GPIO15");



  if (voltage1 < 2 || voltage3 < 2) {
  Serial.print("Voltage are below 2 Volts ");
  Serial.print(voltage1);
  
  } else {
    
  }

  delay(1000); */ // Delay for 1 second
}
