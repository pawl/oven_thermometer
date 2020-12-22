/**
 * oven thermometer
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 * based on the lcdthermocouple example
 * also:
 * - https://randomnerdtutorials.com/esp32-ssd1306-oled-display-arduino-ide/
 * - http://www.esp32learning.com/code/esp32-and-max6675-example.php
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

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

int thermoDO = 19;  // SO
int thermoCS = 23;
int thermoCLK = 5;  // SCK
int thermoVCC = 4;
// int thermoGND = 2;  // attached to GND

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

void setup() {
  Serial.begin(9600);

  // display is using the 3.3v pin, so need to make another pin output 3.3v
  pinMode(thermoVCC, OUTPUT); digitalWrite(thermoVCC, HIGH);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
}

void loop() {
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 1);
  display.println("Thermometer");

  display.setTextSize(5);
  display.setTextColor(WHITE);
  display.setCursor(0, 20);
  uint16_t filteredTemp = getFilteredTemperature(thermocouple, &prevFilteredValue, &kalmanFilter);
  display.print(filteredTemp);
  display.println("F");

  display.display();

  delay(200);
}
