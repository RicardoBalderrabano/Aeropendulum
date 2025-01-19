# Aeropendulum
Aeropendulum - PID control for angular position
# Description

<p align="center">
  <img src="Images_directory/Aeropendulo1_first.gif" alt="Aeropendulum" style="width:320px;height:569px;">
</p>
</br>
This project is centered on the development of a PID controller for the angular position of an aeropendulum. The primary goal was to apply concepts from system dynamics, control theory, and embedded systems programming.

The system's behavior was analyzed by deriving its mathematical model and performing simulations in MATLAB. Control theory principles, including Lyapunov stability, reachability, and controllability theorems, were used to evaluate the system's dynamics and characteristics. These analyses contributed to determining the appropriate sampling time for the system and served as a foundation for implementing a PID control algorithm.

The control logic was implemented using low-level programming techniques, leveraging direct register manipulation for peripheral configuration, including timers, interrupts, PWM signals, I2C, and USART communication protocols. This approach ensured efficient and precise control over the system's performance.

Following the implementation of the control logic, sensor fusion was employed to combine data from the accelerometer and gyroscope. A complementary filter was used to accurately determine the tilt position of the aeropendulum, providing reliable feedback for the control system.

A simple interface was developed in LabVIEW to facilitate system activation and deactivation, as well as to provide real-time feedback on the system's movement.

# Mathematical Model
<img src="Images_directory/MathematicalModel_Aeropendulum.PNG" alt="MathematicalModel_Aeropendulum" style="width:560px;height:569px;">
# System Simulation
<img src="Images_directory/Simulation_Aeropendulum.PNG" alt="Simulation_Aeropendulum" style="width:1200px;height:569px;">
# Hardware Setup
<img src="Images_directory/Hardware_Setup.png" alt="Hardware_Setup_Aeropendulum" style="width:1200px;height:569px;">
# Sampling time
<img src="Images_directory/SamplingTime.png" alt="SamplingTime_Aeropendulum" style="width:1200px;height:569px;">
<img src="Images_directory/Timer_SamplingTime.png" alt="Timer_Aeropendulum" style="width:1200px;height:569px;">
# PID Tunning
<img src="Images_directory/PID_Tunning.png" alt="PID_Tunning_Aeropendulum" style="width:1200px;height:569px;">
<img src="Images_directory/PID_Tunning2.png" alt="PID_Tunning2_Aeropendulum" style="width:1200px;height:569px;">
# HMI Interface
<img src="Images_directory/Interface.png" alt="Interface_Aeropendulum" style="width:582px;height:706px;">
# PID Control Test
<img src="Images_directory/TestPID_Aeropendulum.png" alt="Test_PID_Aeropendulum" style="width:1200px;height:px;">

