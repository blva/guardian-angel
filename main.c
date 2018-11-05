/**
 * Guardian Angel Project
 * Authors: Bianca Lisle
 *          Geraldo Braz
 **/

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <string.h>

/* Definition of input ports */
#define RAIN_PORT 2 //PD2
#define DOOR_PORT 3 //PD3
#define VELOCITY_PORT_ANALOG 0 // IOPORT3

/* Definition of output ports */
#define BUZZER_PORT 4 //PD4
#define MOTOR_PORT 1 //IOPORT2 - PB1 = Pin 9 (PWM)

/* ADC ports */
#define NBR_CHANNELS 1
#define DEPTH 5
#define ADC_CONVERTER_FACTOR 0.00477922077922078 // ((2,5*1.0552519480519482)/552)

/* Structures and variables */

// State Machine
typedef enum{
    normal_state,
    is_door_opened,
    bus_overspeed,
    is_bus_stoped,
    is_raining
}states;


typedef struct sensor_events {
  bool is_door_opened;
  int bus_velocity;
  bool is_bus_stoped;
  bool is_raining;
} sensor_events_t;

volatile uint8_t flag;

/* Threads */

 //FIXME: Study the possibility to create a thread for each sensor event

/* Thread for reading sensors.
 * Read primary sensors:
 * 1. (HP) Velocity speed. -> check speed and speed limit and then activate or not buzzer.
 * 2. (HP) Door sensor. -> check door sensor and if it is opened it is not possible to speed the bus.
 * 3. Rain sensor. -> check raining sensor and reduces limit speed.
 * 4. Ultrasonic sensor -> check ???
 */
static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;

  char buffer[200];

  chRegSetThreadName("read-high-priority-sensors");
  while (true) {
    /* Read velocity speed */
      //This will determine wether the bus has stoped or not. 
      //Do analog reading here

    /* Read Door sensor */
    palReadPad(IOPORT4, RAIN_PORT) == PAL_HIGH ? serial_write("High!\r\n") : serial_write("Low!\r\n");
    chnWrite(&SD1, (const uint8_t *)buffer, strlen(buffer));
    chThdSleepMilliseconds(1000);
  }
}

/* Functions */
void serial_write(char *data) {
  chnWriteTimeout(&SD1, (const uint8_t *)data, strlen(data), TIME_INFINITE);
}

void adc_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n){
  flag = 1;
}

void init_ports() {

  /* Initialize input ports */
  palSetPadMode(IOPORT4, RAIN_PORT, PAL_MODE_INPUT);
  palSetPadMode(IOPORT4, DOOR_PORT, PAL_MODE_INPUT);
  /* Initialize output ports */
  palSetPadMode(IOPORT4, BUZZER_PORT, PAL_MODE_OUTPUT_PUSHPULL); //open drain?
  palSetPadMode(IOPORT2, MOTOR_PORT, PAL_MODE_OUTPUT_PUSHPULL); //open drain?

}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();

  chSysInit();
  init_ports();
  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  serial_write(" Starting Guardian Angel...\r\n");

  /* ADC configuration */
  ADCConfig cfg = {ANALOG_REFERENCE_AVCC};
  ADCConversionGroup group = {0, NBR_CHANNELS, adc_cb, 0x7};
  adcsample_t buffer[DEPTH*NBR_CHANNELS];
  adcStart(&ADCD1, &cfg);

  /*
   * Starts the reading sensors thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);


  while(TRUE) {
    adcStartConversion(&ADCD1, &group, buffer, DEPTH);

    while(!flag){}
      flag =0;

      for(int i = 0; i < DEPTH; i++){
      
        //chprintf((BaseSequentialStream *)&SD1, "%.2f V\n\r",0.1911688311688311*buffer[i]);
        // chprintf((BaseSequentialStream *)&SD1, ">> %d\n\r",buffer[i]);
        chThdSleepMilliseconds(500);
      }
    }

}
