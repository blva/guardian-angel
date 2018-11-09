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
#define SPEED_ANALOG_PORT 0 // IOPORT3

/* Definition of output ports */
#define BUZZER_PORT 4 //PD4
#define MOTOR_PORT 1 //IOPORT2 - PB1 = Pin 9 (PWM)

/* ADC ports */
#define NBR_CHANNELS 1
#define DEPTH 5
#define ADC_CONVERTER_FACTOR 0.00477922077922078 // ((2,5*1.0552519480519482)/552)

/* Structures and variables */
volatile uint8_t flag;
uint8_t maxSpeed;
uint8_t speed = 0;
bool isRanning;
bool doorOpened;
bool overSpeed;

// State Machine
typedef enum{
    normal_state,
    is_door_opened,
    bus_overspeed,
    bus_stopped,
    is_raining
}states;


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

void initPorts() {

  /* Initialize input ports */
  palSetPadMode(IOPORT4, RAIN_PORT, PAL_MODE_INPUT);
  palSetPadMode(IOPORT4, DOOR_PORT, PAL_MODE_INPUT);
  /* Initialize output ports */
  palSetPadMode(IOPORT4, BUZZER_PORT, PAL_MODE_OUTPUT_PUSHPULL); //open drain?
  palSetPadMode(IOPORT2, MOTOR_PORT, PAL_MODE_OUTPUT_PUSHPULL); //open drain?

}


int getSpeed(){
  // FIXME: Implement this methode
  return 10;
}

float speed2DutyCycle(int speed){
  // FIXME: Implement this methode
  return 0.5;
}
void motor_output(float dutyCycle){
  // FIXME: Implement this methode
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

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  initPorts();
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
    states state = bus_stopped; // state default
    doorOpened = true;
    
      // State Machine
      switch(state){
        case bus_stopped:
          if (doorOpened){
             serial_write("Bus Stopped - Door is Open\r\n");

            /* TODO:
              - set PWM to 0%
              
              - Print on serial "Bus Stopped - Door is Open"

            */
          }else{ // Door is close
            serial_write("Bus Stopped - Door is Close\r\n");
            
            if (getSpeed() >= 10){
              state = normal_state;
            }
            /* TODO:
              - The bus can accelerate
              - If speed ultrapass 10km/h go to normal_state
              - Print on serial "Bus Stopped - Door is Close"
            */
          }  
          break;
        case normal_state:
            serial_write("Bus Normal State\r\n");
            int speed = getSpeed();
          
            motor_output(speed2DutyCycle(speed)); 
            
            if (speed > maxSpeed){
              state = bus_overspeed;
            }

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
        
          
          if (getSpeed() < maxSpeed){
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



    // while(!flag){}
    //   flag =0;

    //   for(int i = 0; i < DEPTH; i++){
      
    //     //chprintf((BaseSequentialStream *)&SD1, "%.2f V\n\r",0.1911688311688311*buffer[i]);
    //     // chprintf((BaseSequentialStream *)&SD1, ">> %d\n\r",buffer[i]);
    //     chThdSleepMilliseconds(500);
    //   }
    }

}
