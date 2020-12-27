/**
 * oven thermometer
 *
 * based on:
 * - https://github.com/adafruit/MAX6675-library/blob/1.1.0/examples/lcdthermocouple/lcdthermocouple.ino
 * - https://randomnerdtutorials.com/esp32-ssd1306-oled-display-arduino-ide/
 * - http://www.esp32learning.com/code/esp32-and-max6675-example.php
 *
 * parts:
 * - HiLetgo DC 3-5V MAX6675 Module + K Type Thermocouple Temperature Sensor
 * - UCTRONICS 0.96 Inch OLED Module 12864 128x64 Yellow Blue SSD1306 Driver I2C
 * - ESP32
 */

#include <max6675.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SimpleKalmanFilter.h"
#include "uptime.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define uS_TO_S_FACTOR 1000000LL

// temperature required to wake from deep sleep (oven on)
int WAKE_TEMP = 150;

// amount of time to deep sleep between wake (oven on) temp checks
int DEEPSLEEP_SECONDS = 120;

// time between temp checks, MAX6675 recommends >250ms
int CYCLE_TIME_MS = 250;

// number of cycles before trusting temp calculation
int WARM_UP_CYCLES = 2;

int thermoDO = 19;  // SO
int thermoCS = 23;
int thermoCLK = 5;  // SCK
int thermoVCC = 4;  // is this required?
// int thermoGND = 2;  // attached to GND

// count of temperature checks after deep sleep (oven off)
// TODO: use timestamp?
int cycleCount = 0;

// start with display on to indicate device is on
// use RTC_DATA_ATTR to make this variable survive deep sleep
RTC_DATA_ATTR bool isDisplayOn = true;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

float kalmanEst = 0.4, kalmanQ = 0.01;
SimpleKalmanFilter kalmanFilter(kalmanEst, kalmanEst, kalmanQ);

float filterKalman(float measurement, SimpleKalmanFilter* kalmanFilter)
{
  return kalmanFilter->updateEstimate(measurement);
}

float prevFilteredValue = -1;
float getFilteredTemperature(MAX6675 thermocouple, float* prevValue, SimpleKalmanFilter* kalmanFilter)
{
  // apply kalman filter to temperature data to remove noise
  // based on: https://github.com/hallgeirl/coffee-roaster/blob/4d4fa73c9831b0ff3f28752c454011ab70410de8/src/roaster/roaster.ino#L112
  float reading = thermocouple.readFahrenheit();

  if (*prevValue < 0)
    (*prevValue) = reading;

  float filteredValue = reading;
  filteredValue = filterKalman(reading, kalmanFilter);

  *prevValue = filteredValue;

  return filteredValue;
}

void displayStartMessage() {
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 1);
  display.print("Starting");

  display.display();
}

void setup() {
  Serial.begin(9600);

  // display is using the 3.3v pin, so need to make another pin output 3.3v
  // can't tell if this is actually required by the MAX6675 (it works without it)
  pinMode(thermoVCC, OUTPUT); digitalWrite(thermoVCC, HIGH);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);

  displayStartMessage();
}

void displaySleepMessage() {
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 1);
  display.print("Temp <");
  display.println(WAKE_TEMP);
  display.print("Sleeping...");

  display.display();

  // give user time to read
  delay(5000);
}

void displayInfo(uint16_t filteredTemp) {
  display.clearDisplay();

  // show uptime
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 1);
  display.print("On:");
  uptime::calculateUptime();
  unsigned long uptimeMinutes = uptime::getMinutes();
  if (uptimeMinutes) {
    display.print(uptimeMinutes);
    display.println(" min");
  } else {
    display.print(uptime::getSeconds());
    display.println(" sec");
  }

  // show temperature
  display.setTextSize(5);
  display.setTextColor(WHITE);
  display.setCursor(0, 22);
  display.print(filteredTemp);
  display.println("F");

  display.display();
}

void ensureDisplayOn() {
  if (!isDisplayOn) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    isDisplayOn = true;
  }
}

void ensureDisplayOff() {
  if (isDisplayOn) {
    // power efficient display sleep
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    isDisplayOn = false;
  }
}

void loop() {
  uint16_t filteredTemp = getFilteredTemperature(thermocouple, &prevFilteredValue, &kalmanFilter);

  cycleCount += 1;

  // don't trust the temp reading until after a few cycles
  // TODO: this might be a problem caused by the kalman filter?
  if (cycleCount > WARM_UP_CYCLES) {
    if (filteredTemp > WAKE_TEMP) {
      // oven is on - show info
      ensureDisplayOn();
      displayInfo(filteredTemp);
    } else {
      // oven is off - deep sleep
      cycleCount = 0;  // reset

      // display text before sleeping (to show sensor turned on)
      if (isDisplayOn) {
        displaySleepMessage();
      }

      ensureDisplayOff();

      esp_sleep_enable_timer_wakeup(DEEPSLEEP_SECONDS * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
  }

  delay(CYCLE_TIME_MS);
}
