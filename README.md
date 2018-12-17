# Guardian Angel

This work is a project developed by @blva and @geraldobraz applying the techniques and knowledge aquired on the classes: 
* Real Time Operational Systems Development 
* Embedded System Projects

## What is it?

Guardian Angel is a well known concept used to make buses safer. We developed this using ChibiOS as an RTOS and Arduino Uno as our platform. 

The application has been developed to demonstrate the working of a guardian angel system for buses. The project is composed by the following components:

* GCC Compiler
* Arduino UNO
* Buttons
* DC motor
* Buzzers
* TIP Transistor
* Resistors
* Potenciometer

The system is responsible for safe critical verifications in order to make the bus vehicle safer. This periodically checks the bus velocity and state, guaranteeing that the bus door will not be opened unless the bus is not on movement and that if the bus is on overspeed state a warning will be enabled. Rain detection is another feature, it is responsible for verifying whether is raining or not and if so, the system should warn the driver and reduce the maximum speed limit. 

The usage of an RTOS is important with the usage of threads and interruptions in order to cover all the possible occurrences and guarantee the correct treatment.

## How to run?

First, you need to clone our repository.

    git clone "https://github.com/blva/guardian-angel.git"

Then, on the downloaded folder, you should run:

    make install

This will clone the ChibiOS folder into your repository folder and then you will be able to run a make command. 

    make program

## Specifications

The project implements features envolving Arduino UNO and ChibiOS pheripheral drivers. We have used PWM, Interruptions and ADC conversion.

### Fritzing Circuit
![Fritzing Circuit](https://raw.githubusercontent.com/blva/guardian-angel/master/images/fritzing.png)
### Original Circuit
![Original Circuit](https://raw.githubusercontent.com/blva/guardian-angel/master/images/circuit.png)
### State Machine
![State Machine](https://raw.githubusercontent.com/blva/guardian-angel/master/images/stmachine.png)
