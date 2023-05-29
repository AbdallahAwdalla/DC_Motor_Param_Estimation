
Here is the reformatted version for readme.md:

## Modeling and Simulating the DC Motor System

To model and simulate the DC motor system using Simulink/Simscape, follow these steps:

1. Create a Simulink model of the DC motor system, including the motor, driver, encoder, and microcontroller.
2. Define the system parameters, such as the motor parameters (resistance, inductance, back EMF), the driver parameters (voltage and current limits), and the encoder parameters (resolution, accuracy).
3. Implement the control algorithm in Simulink, based on the program you wrote for the microcontroller or Arduino board.
4. Simulate the system by running the model in Simulink. Apply the same inputs as the experiments you performed on the physical system and observe the simulated response.
![image](https://github.com/AbdallahAwdalla/DC_Motor_Param_Estimation/assets/60265311/21def341-d793-49aa-96de-439118917ea0)
![image](https://github.com/AbdallahAwdalla/DC_Motor_Param_Estimation/assets/60265311/ed029401-0cae-4375-84ee-051f688adc2a)

## Obtaining the Values of the System Parameters

To obtain the values of the system parameters using the Parameter Estimation tool in MATLAB Simulink, follow these steps:

1. Create a MATLAB script or Simulink model that defines the system model and the experimental data.
2. Use the Parameter Estimation tool in Simulink to estimate the values of the system parameters based on the experimental data.
3. Refine the parameter estimates by running additional simulations and comparing the simulated and experimental data.

![image](https://github.com/AbdallahAwdalla/DC_Motor_Param_Estimation/assets/60265311/a42108fc-3baf-4233-9fb9-4c2bd85027ab)
Here is the reformatted version for readme.md:

## Comparing Responses of Physical and Simulated DC Motor Systems

To compare the responses of both the physical system and the simulated one, follow these steps:

1. Run the same experiments on the physical system and the simulated one.
2. Record the experimental data for both systems.
3. Plot the experimental data for both systems and compare the responses.
4. Refine the system model and parameter estimates if there are significant differences between the experimental and simulated data.
![image](https://github.com/AbdallahAwdalla/DC_Motor_Param_Estimation/assets/60265311/5c6caba8-d6e4-4979-860d-2eaa6bd3c1d9)

By following these steps, you can build and test a physical system of a DC motor, model and simulate the system on Simulink/Simscape, and obtain the values of the system parameters using the Parameter Estimation tool in MATLAB Simulink. Comparing the responses of both the physical system and the simulated one will help you validate the accuracy of your model and parameter estimates. This is an important step in designing controllers and predicting the behavior of the system in different scenarios.
## Comment about the accuracy of parameters estimation tool.

As Observed it’s needed for the system parameters to initialized with reasonable value other wise the iterative solution may not be able to get the correct parameters.
