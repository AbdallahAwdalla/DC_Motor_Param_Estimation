# DC_Motor_Param_Estimation 
# Data Acquisition System using KL25Z4
![image](https://github.com/AbdallahAwdalla/DC_Motor_Param_Estimation/assets/60265311/75e7ad21-0e90-41d8-82f3-83805d29b695)

This code implements a Data Acquisition System using the NXP KL25Z4 microcontroller. The system measures the rotational speed of a motor using an encoder and reports the RPM over a serial interface. The system also generates a PWM signal to control the motor speed.

## Prerequisites
To run this code, the following prerequisites are needed:
- NXP KL25Z4 microcontroller
- An encoder connected to pins 4 and 5 of GPIOA
- A motor connected to a PWM output pin of TPM0
- A serial interface for communication

## Code Description
The code consists of several functions that initialize the microcontroller and implement the data acquisition system. Here is a brief description of each function:

### `vPeriodicTask`
This function is a FreeRTOS task that periodically measures the RPM of the motor and sends it over the serial interface.

### `vPeriodicTask2`
This function is a FreeRTOS task that generates a PWM signal with different periods to control the motor speed.

### `encoderISR`
This function is an interrupt service routine that is called when an encoder pulse is detected. It updates the encoder count and direction.

### `initEncoder`
This function initializes the GPIO pins and the SysTick timer for encoder measurements.

### `UART0_Init`
This function initializes UART0 for serial communication.

### `initFreeRTOS`
This function initializes the FreeRTOS scheduler and creates the periodic tasks.

### `getRPM`
This function calculates the RPM of the motor based on the encoder count and the time elapsed between encoder pulses.

The `main` function initializes the microcontroller by updating the system clock, initializing the encoder, UART0, and TPM0. It then starts the FreeRTOS scheduler to execute the periodic tasks.

## Usage
To use this code, follow these steps:
1. Connect the encoder to pins 4 and 5 of GPIOA.
2. Connect the motor to a PWM output pin of TPM0.
3. Connect a serial interface for communication.
4. Compile and flash the code to the KL25Z4 microcontroller.
5. Monitor the serial interface to view the RPM values.

## Contributing
Contributions to this project are welcome. To contribute, please follow these steps:
1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Implement your changes.
4. Test your changes.
5. Create a pull request with a detailed description of your changes.

## License
This code is licensed under the MIT License. See the LICENSE file for more information.

