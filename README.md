# DC_Motor_Param_Estimation
![image](https://github.com/AbdallahAwdalla/DC_Motor_Param_Estimation/assets/60265311/d7eb9874-2ab5-45f5-b4c1-0cb20d8c6138)

To build a physical DC motor system and derive its transfer function, follow these steps:

**Building the Physical DC Motor System**
1. Mount the DC motor on a stable base or chassis. Make sure it is securely fixed and can rotate freely.
2. Connect the power supply to the motor driver. The voltage and current ratings of the power supply should be compatible with the motor and driver specifications.
3. Connect the motor driver to the DC motor. Make sure the polarity of the connections is correct.
4. Connect the encoder to the motor shaft. The encoder will provide feedback on the motor position and speed.
5. Connect the motor driver and encoder to the microcontroller or Arduino board. The board will control the motor driver and read the encoder signals.
6. Write a program in the microcontroller to control the motor speed and direction based on the encoder feedback. You can use PWM (Pulse Width Modulation) to control the motor speed.
7. Test the system by running the program and observing the motor behavior. You can use a multimeter or oscilloscope to measure the voltage and current of the motor and the encoder signals.

**Deriving the Transfer Function**
1. Derive the equations of motion for the motor, which relate the torque and angular acceleration of the motor to the voltage and current applied to it.
2. Linearize the equations around an operating point to obtain a linear model of the system.
3. Take the Laplace transform of the linearized equations to obtainthe transfer function in the s-domain.

The resulting transfer function will have the form:
 
![image](https://github.com/AbdallahAwdalla/DC_Motor_Param_Estimation/assets/60265311/28c3c820-1513-472e-ac4e-487ca314ed99)
