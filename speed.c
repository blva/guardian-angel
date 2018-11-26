#include <ch.h>
#include <hal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <chprintf.h>

#define MAXIMUM_VOLTAGE 5
#define MINIMUM_VOLTAGE 0.5
#define BUS_MAXIMUM_SUPPORTED_SPEED 100 /* Bus speed in km/h */
#define ADC_TO_SPEED_CONV_VALUE BUS_MAXIMUM_SUPPORTED_SPEED/MAXIMUM_VOLTAGE - MINIMUM_VOLTAGE

int get_speed(uint16_t adc_value) {
  char buffer [sizeof(uint16_t)*8+1];
  uint16_t number = (adc_value) * (ADC_TO_SPEED_CONV_VALUE);
  ltoa(number, buffer, 10);
  serial_write("speed:");
  serial_write(buffer);
  return number;
}

int is_speed_above_limit(int speed, int limit) {
  int ret = 1;
  if (speed <= limit) {
    ret = 0;
  }

  return ret;
}