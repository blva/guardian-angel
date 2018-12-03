/**
 * Guardian Angel Project
 * Authors: Bianca Lisle
 *          Geraldo Braz
 **/

#include "ch.h"
#include "hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "chprintf.h"
#include "speed.h"

/* Debug Flags */
#define DEBUG 1

/* Definition of input ports */
#define RAIN_PORT 2 //PD2
#define DOOR_PORT 3 //PD3
#define SPEED_ANALOG_PORT 0 // IOPORT3

/* Definition of output ports */
#define BUZZER_PORT 4 //PD4
#define MOTOR_PORT 2 //IOPORT2 - PB1 = Pin 10 (PWM)

/* ADC ports */
#define NBR_CHANNELS 1
#define DEPTH 5
#define ADC_CONVERTER_FACTOR 0.19116883116883118 // ((100*1.0552519480519482)/552)

BaseSequentialStream* chp = (BaseSequentialStream*) &SD1;


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
int rainFlag = 0;
int currentMaxSpeed = 195;
int maxSpeed = 195; /* Fixed Speed */
int speed = 0;
int init_flag = 0;
float adc_to_speed_cvalue; 

bool isRanning;
int doorOpened;
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

void serial_write(void *data) {
  if (DEBUG) {
    chprintf(chp, "%s\n\r",data);
  }
}

/* Callback functions */
volatile uint8_t got_adc;
void adc_cb(ADCDriver *adcp, adcsample_t *bufferADC, size_t n) {
  got_adc = 1;
}

//  ********* Interruptions *********
static void rainButton_cb(EXTDriver *extp, expchannel_t channel){
  (void)extp;
  (void)channel;

  chSysLockFromISR();  
  palTogglePad(IOPORT2, PORTB_LED1);
  rainFlag = (currentMaxSpeed == maxSpeed) ? 1 : 0;

  chSysUnlockFromISR();
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


void setMaxSpeed(int speed){
    currentMaxSpeed = speed;
}

float speed2DutyCycle(int speed){
  /* README 
      speed             duty cycle
       100   -----------   100%
       20    -----------    1%
   inputSpeed ---------- outputDutyCycle

    outputDutyCycle =  inputSpeed/20

  */ 
 return speed * 100;
}

void motor_output(float dutyCycle){
  // int step = 6;
  // int width = step;
  // TODO: Print the Duty Cycle
  char buffer[10];
  ltoa(PWM_FRACTION_TO_WIDTH(&PWMD1, 100, speed), buffer, 10);
  pwmEnableChannel(&PWMD1, 1, PWM_FRACTION_TO_WIDTH(&PWMD1, 100, speed));
  /*
    width += step;
  if ((width >= 0x3FF) || (width < 10)) {
      width -= step;
      step = -step;
  }  
  */
  
}

void buzzer_output(int state){
    switch (state){
        case 0: // Opened door
            palTogglePad(IOPORT4, BUZZER_PORT);
            chThdSleepMilliseconds(500);
            break;
        case 1: // Over Speed
            palTogglePad(IOPORT4, BUZZER_PORT);
            chThdSleepMilliseconds(100);
            break;
    }

}

/* Thread for reading door command */
static THD_WORKING_AREA(waOpenDoorCommand, 32);
static THD_FUNCTION(OpenDoorCommand, arg) {
  (void)arg;
  chRegSetThreadName("open-door-thread");
  int door_command;
  while (true) {
    door_command = (palReadPad(IOPORT4,DOOR_PORT) == PAL_LOW) ? 1 : 0;
    /* Open the door onnly if the bus has stopped! */
    if (door_command && (state <= waiting_acceleration)) {
      doorOpened ^= 1;
    }
    else if (door_command) {
      serial_write("Door can't be opened!");
    }
    chThdSleepMilliseconds(500);
  }
}

/* Thread for reading open door command */
static THD_WORKING_AREA(waReadSpeed, 32);
static THD_FUNCTION(readSpeed, arg) {
  (void)arg;
  chRegSetThreadName("read-speed-thread");
  uint16_t adc_value;

  while (true) {
    if (got_adc) {
      adc_value = get_adc_conversion(arg);
      speed = get_speed(adc_value);
    }
    chThdSleepMilliseconds(300);
  }
}

void st_machine(void) {
  int ret;

  //TODO: add bus break threatment to identify if bus has stopped again!

  /* Starts State Machine */
  switch(state){
    case bus_stopped:
      /* Functionalities:
        - set PWM to 0%
        - Print on serial "Bus Stopped - Door is Open"
        - Do not allow the motor to run
      */
      serial_write("Bus Stopped - Waiting for closed door");

      if (!doorOpened) {
        state = waiting_acceleration;
      }
      else {
        /* If door is opened, keep the bus stoped */
        //motor_output(0); // Turning off the motor
        buzzer_output(0); //
        serial_write("Please close the door!");
      }
      break;
    case waiting_acceleration:
      /* Functionalities:
        - Allow motor powering
        - The bus can accelerate
        - If speed ultrapass 10km/h go to normal_state
        - If door is opened, goes back to bus_stopped
      */
      serial_write("Door closed, waiting for acceleration");
      if (doorOpened) {
        state = bus_stopped;
      }
      ret = is_speed_above_limit(speed, 25);
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
      palWritePad(IOPORT4,BUZZER_PORT,0);
      serial_write("Bus Normal State");
      motor_output(speed2DutyCycle(speed));
      ret = is_speed_above_limit(speed, currentMaxSpeed);
      state = (ret) ? overspeed_alert : normal_state;
      if (rainFlag){
        state = rain_alert;
      }
      break;
    case rain_alert:
      /* Functionalities:
          - Update maximum speed
          - Go back to normal state, if it is still raining,
            then the state will be reloaded until it is over. 
      */
      serial_write("Warning! - It is Ranning, maximum speed has changed");    
      setMaxSpeed(80);
      motor_output(speed2DutyCycle(speed));

      if (!rainFlag) {
        state = normal_state;
        setMaxSpeed(maxSpeed);
      }
      break;
    case overspeed_alert:
      /* Functionalities:
        - Turn the buzzer to High                  
        - Only leave state after overspeed is over
      */
      serial_write("Warning! - Overspeed");
      buzzer_output(1); 
      ret = is_speed_above_limit(speed, currentMaxSpeed);
      state = (ret) ? overspeed_alert : normal_state;
      break;  
    default:
      state = bus_stopped;
      break;
  }
}

int main(void) {

  /* PWM Config */
  static PWMConfig pwmcfg = {
          0, 0xFF, 0,
          {{PWM_OUTPUT_DISABLED, 0}, {PWM_OUTPUT_ACTIVE_HIGH, 0}}
  };

  /* Interruption Config */
  static const EXTConfig extcfg = {
    {
      {EXT_CH_MODE_FALLING_EDGE , rainButton_cb},      /* INT0 Config. */
      {EXT_CH_MODE_DISABLED , NULL},      /* INT1 Config. */
      {EXT_CH_MODE_DISABLED , NULL},      /* INT2 Config. */
      {EXT_CH_MODE_DISABLED , NULL},      /* INT3 Config. */
    }
  };

  /* ADC Config */
  ADCConfig cfg = {ANALOG_REFERENCE_AVCC};
  ADCConversionGroup group = {0, NBR_CHANNELS, adc_cb, 0x7};
  adcsample_t bufferADC[DEPTH*NBR_CHANNELS];
  halInit();
  chSysInit();
  initPorts();

  /* Start peripherals */
  pwmStart(&PWMD1, &pwmcfg);
  adcStart(&ADCD1, &cfg);
  sdStart(&SD1, NULL);
  extStart(&EXTD1, &extcfg);
  extChannelEnable(&EXTD1, INT0); // PD2 (4)

  serial_write("Guardian Angel");

  /* Initialize Threads */
  chThdCreateStatic(waReadSpeed, sizeof(waReadSpeed), NORMALPRIO, readSpeed, &bufferADC);
  chThdCreateStatic(waOpenDoorCommand, sizeof(waOpenDoorCommand), NORMALPRIO, OpenDoorCommand, NULL);

  /* Init state */
  state = bus_stopped;
  doorOpened = 1; /* Door starts closed */

  palWritePad(IOPORT4,BUZZER_PORT,0);
  palWritePad(IOPORT2,MOTOR_PORT,0);

  while(TRUE) {
    /* Start ADC conversion */
    adcStartConversion(&ADCD1, &group, bufferADC, DEPTH);
    /* Run state machine */
    st_machine();
    /* Sleep time of the "main" thread */
    chThdSleepMilliseconds(200);
  }
}
