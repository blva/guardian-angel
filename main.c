/**
 * Guardian Angel Project
 * Authors: Bianca Lisle
 *          Geraldo Braz
 **/

#include <ch.h>
#include <hal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <chprintf.h>

/* Debug Flags */
#define DEBUG 1

/* Definition of input ports */
#define RAIN_PORT 2 //PD2
#define DOOR_PORT 3 //PD3
#define SPEED_ANALOG_PORT 0 // IOPORT3

/* Definition of output ports */
#define BUZZER_PORT 4 //PD4
#define MOTOR_PORT 1 //IOPORT2 - PB1 = Pin 9 (PWM)

/* ADC ports */
#define NBR_CHANNELS 1
#define DEPTH 5
#define ADC_CONVERTER_FACTOR 0.00477922077922078 // ((2,5*1.0552519480519482)/552)
#define MAXIMUM_VOLTAGE 5
#define MINIMUM_VOLTAGE 0.5
#define BUS_MAXIMUM_SUPPORTED_SPEED 100 /* Bus speed in km/h */

float get_adc_convert_value(void) {
  return BUS_MAXIMUM_SUPPORTED_SPEED/(MAXIMUM_VOLTAGE - MINIMUM_VOLTAGE);
}

// State Machine
typedef enum{
    bus_stopped,
    waiting_acceleration,
    rain_alert,
    overspeed_alert,
    normal_state,
    bluetooth_pairing
}states;

/* Structures and variables */
volatile uint8_t flag;
int maxSpeed = 100;
int speed = 0;
float adc_to_speed_cvalue; 

bool isRanning;
bool doorOpened;
bool overSpeed;

states state;

/*  Config */
void initPorts(void) {

  /* Initialize input ports */
  palSetPadMode(IOPORT4, RAIN_PORT, PAL_MODE_INPUT);
  palSetPadMode(IOPORT4, DOOR_PORT, PAL_MODE_INPUT);
  /* Initialize output ports */
  palSetPadMode(IOPORT4, BUZZER_PORT, PAL_MODE_OUTPUT_PUSHPULL); //open drain?
  palSetPadMode(IOPORT2, MOTOR_PORT, PAL_MODE_OUTPUT_PUSHPULL); //open drain?

}

int forward_door_command(int command) {
  /* This function forwards the door command to the bus, otherwise, the door is not opened */
  bool door_opened = command;
  return 1;
}

void serial_write(void *data) {
  if (DEBUG) {
    chprintf((BaseSequentialStream *)&SD1, "%s\n\r",data);
  }
}

/* Callback functions */
volatile uint8_t got_adc;
void adc_cb(ADCDriver *adcp, adcsample_t *bufferADC, size_t n) {
  got_adc = 1;
}

// TODO: Create a interruption that checks if the door is open.
void doorOpened_cb(void){
  // state = waiting_acceleration;
}


//  ********* Threads *********
void rainButton_cb(void){
  state = rain_alert;
}

void overSpeed_cb(void){
  state = overspeed_alert;
}
//  ***************************

/* Aux Functions */

uint64_t get_adc_conversion(adcsample_t *bufferADC) {
  uint16_t number = ADC_CONVERTER_FACTOR * bufferADC[0];
  got_adc = 0;
  return number;
}

void get_speed(uint16_t adcValue) {
  char buffer [sizeof(uint16_t)*8+1];
  uint16_t number = (adcValue) * (adc_to_speed_cvalue);
  ltoa(number, buffer, 10);
  serial_write("read speed:");
  serial_write(buffer);
  speed = adcValue*adc_to_speed_cvalue;
}

void setMaxSpeed(int speed){
    maxSpeed = speed;
}

float speed2DutyCycle(int speed){
  // FIXME: Implement this methode
  return 0.5;
}



void motor_output(float dutyCycle){
  int step = 6;
  int width = step;

  // TODO: Print the Duty Cycle
  // serial_write("Pwm! \r\n");
  pwmEnableChannel(&PWMD1, 1, width);
  width += step;
  if ((width >= 0x3FF) || (width < 10)) {
      width -= step;
      step = -step;
  }
}

void buzzer_output(int state){
    switch (state){
        case 0: // Opened door
            break;
        case 1: // Raining
            break;
    }

}

/**
 * @brief   Identifies wether speed is above limit ot not.
 *
 * @param[in] speed    Integer to indicate speed value
 * @param[in] limit    Integer to indicate limit to compair
 * @return  Null if speed is NOT above limit and 1 if speed is above limit
 */
int is_speed_above_limit(int speed, int limit) {
  int ret = 1;
  if (speed <= limit) {
    ret = 0;
  }

  return ret;
}

/* Threads */

 // FIXME: Study the possibility to create a thread for each sensor event

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
  chRegSetThreadName("read-speed-thread");
  uint16_t adc_value;

  while (true) {
    if (got_adc) {
      adc_value = get_adc_conversion(arg);
      get_speed(adc_value);
    }
    chThdSleepMilliseconds(1000);
  }
}

void st_machine(adcsample_t *bufferADC) {
  uint16_t adc_value = 0;
  int ret;

  /* Starts State Machine */
  switch(state){
    case bus_stopped:
      /* Functionalities:
        - set PWM to 0%
        - Print on serial "Bus Stopped - Door is Open"
        - Do not allow the motor to run
      */
      serial_write("Bus Stopped - Waiting for closed door\r\n");
      //motor_output(0); // Turning off the motor
      //buzzer_output(0); //
      
      //FIXME: Delete me after
      state = waiting_acceleration;
      //
      break;
    case waiting_acceleration:
      /* Functionalities:
        - Allow motor powering
        - The bus can accelerate
        - If speed ultrapass 10km/h go to normal_state
        - If door is opened, goes back to bus_stopped
        - Print on serial "Door closed, waiting for acceleration"
      */
      
      //TODO -> add motor output
      serial_write("Door closed, waiting for acceleration\r\n");

      ret = is_speed_above_limit(speed, 10);
      state = (ret) ? normal_state : waiting_acceleration;

      break;
    case normal_state:
      /* Functionalities:
        - The bus can accelerate
        - Relate the ADC with the duty cycle
        - Turn On the motor (PWM)
        - If the speed ultrapass the limit go to another state
        - Print on serial "Bus Normal State"
      */
      serial_write("Bus Normal State\r\n");
     // motor_output(speed2DutyCycle(get_speed(bufferADC))); // Get the analog value of the speed and converts it to duty cycle

      break;
    case rain_alert:
      /* Functionalities:
          - Check if rain stopped or started
          - Turn the buzzer to High
          - Change speed limit        
      */

      /* Check wether rain is over or just started! */
      serial_write("Warning! - It is Ranning\r\n");    
      /* Setting new speed limit */
      // setMaxSpeed(80);
      /* Go back to normal state */ 
      state = normal_state;
      break;

    case overspeed_alert:
      /* Functionalities:
        - Turn the buzzer to High                  
        - Print on serial "Warning! - Overspeed"
        - Only leave state after overspeed
      */
      serial_write("Warning! - Overspeed\r\n");
      // buzzer_output(1); // 
      break;  
    default:
      state = bus_stopped;
      break;
  }
}

int main(void) {

  /* PWM Config */
  static PWMConfig pwmcfg = {
          0, 0x3FF, 0,
          {{PWM_OUTPUT_DISABLED, 0}, {PWM_OUTPUT_ACTIVE_HIGH, 0}}
  };

  /* ADC Config */
  ADCConfig cfg = {ANALOG_REFERENCE_AVCC};
  ADCConversionGroup group = {0, NBR_CHANNELS, adc_cb, 0x7};
  adcsample_t bufferADC[DEPTH*NBR_CHANNELS];

  halInit();
  chSysInit();
  initPorts();


  pwmStart(&PWMD1, &pwmcfg);
  sdStart(&SD1, NULL);

  serial_write("Guardian Angel \r\n");

  adcStart(&ADCD1, &cfg);
  adc_to_speed_cvalue = get_adc_convert_value();

  /*
   * Starts the reading sensors thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, &bufferADC);

  state = bus_stopped; // state default
  doorOpened = true;

  while(TRUE) {
    /* Start ADC conversion */
    adcStartConversion(&ADCD1, &group, bufferADC, DEPTH);
    /* Run state machine */
    st_machine(bufferADC);
    /* Sleep time of the "main" thread */
    chThdSleepMilliseconds(3000);
  }
}
