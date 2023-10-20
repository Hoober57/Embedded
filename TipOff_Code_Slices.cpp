/*  ######################################################################################################################################
o program the ESP32-S2 real-time clock (RTC), you can follow these steps:

Start by installing the Arduino IDE. You can download it from the official website at https://www.arduino.cc/en/software.

Open the Arduino IDE and go to "File" > "Preferences." In the "Additional Boards Manager URLs" field, add the following 
URL: https://dl.espressif.com/dl/package_esp32_index.json. Click "OK" to save the settings.

Next, install the ESP32 boards package. Go to "Tools" > "Board" > "Boards Manager." In the "Filter your search..." field, 
search for "esp32" and install the "esp32" package by Espressif Systems.

Now, create a new Arduino sketch by going to "File" > "New."

In the new sketch, include the necessary libraries at the beginning. Copy and paste the following lines of code:  
############################################################################################################################################*/

#include <WiFi.h> 
#include <WiFiUdp.h>
#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "esp_task_wdt.h"


void setup() {
  // Initialize the RTC
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_2M);
  rtc_clk_apb_freq_update(RTC_CLK_FREQ_8M);
  rtc_clk_apb_freq_set(RTC_CLK_FREQ_32K);
  rtc_gpio_init(GPIO_NUM_37);
  rtc_gpio_set_direction(GPIO_NUM_37, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_hold_dis(GPIO_NUM_37);
}

void loop() {
  // Get the current time from RTC
  time_t currentTime = time(nullptr);
  
  // Display the current time
  Serial.print("Current time: ");
  Serial.println(ctime(&currentTime));

  delay(1000); // Wait for 1 second
}

/*###############################################################################################################################################

Here's an Arduino program that uses RainMaker to turn ON and OFF GPIO9 and measures temperature on an analog input on GPIO10 for an ESP32-S2:

################################################################################################################################################*/


#include <esp_rainmaker.h>
#include <driver/gpio.h>
#include <driver/adc.h>

#define GPIO_PIN 9
#define ADC_PIN 10

// Callback function for turning ON GPIO9
void turnOn(void *arg, esp_rmaker_property_t *property, esp_rmaker_set_params_t *params)
{
    gpio_set_level(GPIO_PIN, 1);
}

// Callback function for turning OFF GPIO9
void turnOff(void *arg, esp_rmaker_property_t *property, esp_rmaker_set_params_t *params)
{
    gpio_set_level(GPIO_PIN, 0);
}

// Callback function for measuring temperature
float measureTemperature(void)
{
    uint32_t adc_value = adc1_get_raw(ADC1_CHANNEL_0);
    float temperature = convertToTemperature(adc_value); // Implement your temperature conversion logic here
    return temperature;
}

// Initialize RainMaker
void initRainMaker()
{
    esp_rmaker_init();

    // Set GPIO9 as an output
    gpio_pad_select_gpio(GPIO_PIN);
    gpio_set_direction(GPIO_PIN, GPIO_MODE_OUTPUT);

    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

    // Create ON and OFF properties for GPIO9 and add them to the device
    esp_rmaker_device_add_button("gpio9_on", turnOn, NULL);
    esp_rmaker_device_add_button("gpio9_off", turnOff, NULL);

    // Add temperature sensor property to the device
    esp_rmaker_device_add_temperature_sensor("temperature", NULL, measureTemperature, NULL);

    // Start the RainMaker task
    esp_rmaker_start();
}

void setup()
{
    initRainMaker();
}

void loop()
{
    // Nothing to do in the loop
}


/*#####################################################################################################################

n this program, we include the necessary libraries for ​RainMaker, GPIO, and ADC functionality. We define the GPIO_PIN as 9 and ADC_PIN as 10.

We then create callback functions for turning ON and OFF ​GPIO9 using the gpio_set_level function. We also create a callback function for 
measuring temperature using the ADC. Please note that you need to implement your own temperature conversion logic in the measureTemperature function.

In the initRainMaker function, we initialize RainMaker, set GPIO9 as an output, configure the ADC, and add properties for turning ON and OFF the GPIO9 pin.
 We also add a property for the temperature sensor.

In the setup function, we call the initRainMaker function to initialize RainMaker.

Finally, in the loop function, there is nothing to do as RainMaker handles all the device management and property updates.

Please make sure to customize the program according to your specific GPIO pin and ADC configuration. This program should provide a starting 
point for using RainMaker to control GPIO9 and measure temperature on an analog input on ​GPIO10 for an ​ESP32-S2 board.


######################################################################################################################*/

/*#######################################################################################################################

To create a program for ESP32-S2 using Rainmaker that connects to Wi-Fi, requests the current time from an NTP server, and sends the current time 
to the Serial Monitor whenever a button connected to GPIO09 is pressed, follow these steps:

Set up the ESP32-S2: Install the necessary drivers and development tools, such as the Arduino IDE or PlatformIO, and ensure that
 your board is properly connected to your computer.

Create a Rainmaker account: Go to the Rainmaker website (https://rainmaker.espressif.com/) and create an account.

Create a Rainmaker product: Once you have an account, create a new Rainmaker product to represent your ESP32-S2 device.

Configure Wi-Fi credentials: In the Rainmaker product configuration, provide the Wi-Fi credentials (SSID and password) for your Wi-Fi network.

Generate the Rainmaker code: After configuring the Wi-Fi credentials, Rainmaker will generate the necessary code for the ESP32-S2.

Integrate Rainmaker code: Copy the generated code and integrate it into your ESP32-S2 project. This typically involves adding the
Rainmaker library, initializing the Rainmaker service, and handling the device configuration.

Request time from NTP server: Use the built-in ntp_client function to request the current time from an NTP server. Specify the NTP server
URL or IP address in the function call.

Read button status: Use the Rainmaker GPIO library to read the status of the button connected to GPIO09. You can use the is_pressed method to check if the button is pressed.

Send current time to Serial Monitor: In the button press event handler, get the current time using the localtime function and 
convert it to a string format. Then, send the string to the Serial Monitor using the appropriate UART communication function.

Here's an example program that demonstrates the above steps:

*/

from rainmaker import ntp_client
import machine
import utime
import network

# Connect to the Internet
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect("<SSID>", "<PASSWORD>")

# Wait for the device to connect to the Internet
while not sta_if.isconnected():
    pass

# Initialize the UART
uart = machine.UART(0, 115200)

# Enable the UART
uart.init(115200, bits=8, parity=None, stop=1, pins=("P3", "P4"))

# Initialize GPIO pins
button_pin = machine.Pin(9, machine.Pin.IN)

def request_current_time():
    # Request current time from NTP server
    current_time = ntp_client.request_time("<NTP_SERVER>")
    return current_time

def read_button_status():
    # Read button status
    status = button_pin.value()
    return status

def send_time_to_serial_monitor(current_time):
    # Convert time to string format
    time_string = utime.strftime("%Y-%m-%d %H:%M:%S", utime.localtime(current_time))
    # Send time to Serial Monitor
    uart.write(time_string.encode())

# Main loop
while True:
    # Check if the button is pressed
    if read_button_status() == 1:
        # Request current time
        current_time = request_current_time()
        
        # Send time to Serial Monitor
        send_time_to_serial_monitor(current_time)
		
/*#################################################################################################################

Make sure to replace <SSID>, <PASSWORD>, and <NTP_SERVER> with appropriate values for your ​Wi-Fi network and desired ​NTP server.

By following these steps and using the example program, you can connect your ​ESP32-S2 to Wi-Fi, request the 
current time from an NTP server, and send the current time to the Serial Monitor whenever the button connected to ​GPIO09 is pressed.	

######################################################################################################################*/

