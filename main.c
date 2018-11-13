/**
 * Guardian Angel Project
 * Authors: Bianca Lisle
 *          Geraldo Braz
 **/
#include <ch.h>
#include <hal.h>
#include <string.h>
#include <stdio.h>

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

// State Machine
typedef enum{
    normal_state,
    is_door_opened,
    bus_overspeed,
    bus_stopped,
    is_raining
}states;

/* Structures and variables */
volatile uint8_t flag;
uint8_t maxSpeed = 100;
uint8_t speed = 0;
bool isRanning;
bool doorOpened;
bool overSpeed;
char bufferADC[200];
states state;

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

  

  chRegSetThreadName("read-high-priority-sensors");
  while (true) {
    /* Read velocity speed */
      //This will determine wether the bus has stoped or not. 
      //Do analog reading here

    /* Read Door sensor */
    // palReadPad(IOPORT4, RAIN_PORT) == PAL_HIGH ? serial_write("High!\r\n") : serial_write("Low!\r\n");
    //chnWrite(&SD1, (const uint8_t *)buffer, strlen(buffer));
    chThdSleepMilliseconds(5000);
  }
}



/*  Config */
void initPorts() {

  /* Initialize input ports */
  palSetPadMode(IOPORT4, RAIN_PORT, PAL_MODE_INPUT);
  palSetPadMode(IOPORT4, DOOR_PORT, PAL_MODE_INPUT);
  /* Initialize output ports */
  palSetPadMode(IOPORT4, BUZZER_PORT, PAL_MODE_OUTPUT_PUSHPULL); //open drain?
  palSetPadMode(IOPORT2, MOTOR_PORT, PAL_MODE_OUTPUT_PUSHPULL); //open drain?

}

// Callback Fuctions
void adc_cb(ADCDriver *adcp, adcsample_t *bufferADC, size_t n){
  // FIXME: Print de speed! 
  if (getSpeed(bufferADC[0]) >= maxSpeed){
      state = bus_overspeed;
  }

}

void rainButton_cb(){
  state = is_raining;
}

void doorOpened_cb(){
  state = is_door_opened;
}

void overSpeed_cb(){
  state = bus_overspeed;
}

/* Aux Functions */
void serial_write(char *data) {
  chnWriteTimeout(&SD1, (const uint8_t *)data, strlen(data), TIME_INFINITE);
}

int getSpeed(char buffer){
  return (ADC_CONVERTER_FACTOR*buffer);
}

float speed2DutyCycle(int speed){
  // FIXME: Implement this methode
  return 0.5;
}

void motor_output(float dutyCycle){
  int step = 6;
  int width = step;

  // TODO: Print the Duty Cycle
  serial_write("Pwm! \r\n");
  pwmEnableChannel(&PWMD1, 1, width);
  width += step;
  if ((width >= 0x3FF) || (width < 10)) {
      width -= step;
      step = -step;
  }
}

void buzzer_output(int state){
  // FIXME: Implement this methode
  //  If state is:
  //  1: Play the song number 1 (Door Opened )
  //  2: Play the song number 2 (Over Speed)
}

/*
 * Application entry point.
 */
int main(void) {

  // PWM Config
  static PWMConfig pwmcfg = {
          0, 0x3FF, 0,
          {{PWM_OUTPUT_DISABLED, 0}, {PWM_OUTPUT_ACTIVE_HIGH, 0}}
  };

  // ADC Config
  ADCConfig cfg = {ANALOG_REFERENCE_AVCC};
  ADCConversionGroup group = {0, NBR_CHANNELS, adc_cb, 0x7};
  adcsample_t bufferADC[DEPTH*NBR_CHANNELS];

  halInit();
  chSysInit();
  initPorts();


  pwmStart(&PWMD1, &pwmcfg);
  sdStart(&SD1, NULL);
  adcStart(&ADCD1, &cfg);

  serial_write(" Starting Guardian Angel...\r\n");


  /*
   * Starts the reading sensors thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  state = bus_stopped; // state default
  doorOpened = true;

  while(TRUE) {
      adcStartConversion(&ADCD1, &group, bufferADC, DEPTH);
      // State Machine
      switch(state){
        case bus_stopped:
          if (doorOpened){
            serial_write("Bus Stopped - Door is Opened\r\n");
            motor_output(0);
            doorOpened = 0;// FIXME: Delete this line later 
            /* TODO:
              - set PWM to 0%
              - Print on serial "Bus Stopped - Door is Open"
              - Do not allow the motor to run
            */
          } else { // Door is closed
            serial_write("Bus Stopped - Door is Closed\r\n");
            // TODO: Check the speed value!
            if (getSpeed(bufferADC[0]) >= 10){
              state = normal_state;
            }
            /* TODO:
              - Allow motor powering
              - The bus can accelerate
              - If speed ultrapass 10km/h go to normal_state
              - Print on serial "Bus Stopped - Door is Close"
            */
          }  
          break;
        case normal_state:
            serial_write("Bus Normal State\r\n");
            int speed = getSpeed(bufferADC);
            motor_output(speed2DutyCycle(speed)); 
            
            

            /* TODO:
              - The bus can accelerate
              - Relate the ADC with the duty cycle
              - Turn On the motor (PWM)
              - If the speed ultrapass the limit go to another state
              - Print on serial "Bus Normal State"
            */
          break;
        case is_door_opened:
            serial_write("Warning! - Door Opened\r\n");
            // Turning off the motor
            motor_output(0); 
            state = bus_stopped;

          /* TODO:
              - Turn the buzzer to High            
              - Turn off the motor 
              - Go to bus_stopped state        
              - Print on serial "Warning! - Door Opened"
            */
          break;
        case bus_overspeed:
          serial_write("Warning! - Overspeed\r\n");
          buzzer_output(2);
        
          if (getSpeed(bufferADC[0]) < maxSpeed){
            state = normal_state;
          }
          break;  

          /* TODO:
              - Turn the buzzer to High                  
              - Print on serial "Warning! - Overspeed"
            */
        case is_raining:
          
          break;
        default:
          state = bus_stopped;
          break;


      }

  chThdSleepMilliseconds(3000);


    // while(!flag){}
    //   flag =0;

    //   for(int i = 0; i < DEPTH; i++){
      
    //     //chprintf((BaseSequentialStream *)&SD1, "%.2f V\n\r",0.1911688311688311*buffer[i]);
    //     // chprintf((BaseSequentialStream *)&SD1, ">> %d\n\r",buffer[i]);
    //   }
    }

}
