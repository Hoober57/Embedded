#include <driver/ledc.h>

#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8
#define PWM_FREQUENCY 2700
#define PWM_DUTY_CYCLE 128  // 50% duty cycle (0-255)
#define PWM_GPIO_PIN 9

void setup() {
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_GPIO_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, PWM_DUTY_CYCLE);
  delay(3000);
}

void loop() {
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_GPIO_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, PWM_DUTY_CYCLE);
  delay(100);
  ledcWrite(PWM_CHANNEL, 0);
  delay(100);

  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_GPIO_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, PWM_DUTY_CYCLE);
  delay(150);
  ledcWrite(PWM_CHANNEL, 0);
  delay(100);

  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_GPIO_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, PWM_DUTY_CYCLE);
  delay(200);
  ledcWrite(PWM_CHANNEL, 0);
  delay(300);



/*
  ledcSetup(PWM_CHANNEL, 2850, PWM_RESOLUTION);
  ledcAttachPin(PWM_GPIO_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, PWM_DUTY_CYCLE);
  delay(1500);
  ledcWrite(PWM_CHANNEL, 0);

  ledcSetup(PWM_CHANNEL, 2400, PWM_RESOLUTION);
  ledcAttachPin(PWM_GPIO_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, PWM_DUTY_CYCLE);
  delay(1000);
  ledcWrite(PWM_CHANNEL, 0);
*/
}// Add any additional code here

