# Smash It

## About
* **SmashIt** is a tennis playing robot designed to autonomously return incoming tennis balls.
* The robot is a **mobile base with a mounted robotic arm**.
    * **The Base:** The base is a 4 wheel mecanum drive setup.
    * **The Arm:** The arm consist of 2 joints.
      *  Joint 1 is a spherical joint that allows rotation in three degrees of freedom: X, Y, and Z.
      *  The second joint is a simple revolute joint. 
* The project is implemented in both simulation (using Gazebo) and real-world hardware, utilizing ROS2 as the middleware framework.
  
>  [!NOTE]
> This project is divided into 2 parts, Simulation and Real world. The main branch is kept empty for clarity. 
> * Simulation: This part is software based only and uses Gazebo as simulator. Switch to 'simulation' branch to checkout its code.
> * Realworld: This part utilize realworld hardware instead of simulation. Switch to 'real_world' branch to checkout its code.


## Simulation

https://github.com/user-attachments/assets/51eb29b0-6327-4e09-ad26-5f32ef586e22



## Real World(Work in Progress)
A real world model os "SmashIt" is also being worked upon. As of now, the base(4 wheel cart) is completed and is fully functional. It can navigate to a specified coordinate within a given time, provided the input is feasible. However, due to the absence of external references like GPS, vision, or LiDAR, positional drift occurs over time. Later, an IR-based stereo vision setup will be added as an external reference. The follwoing are key highlits of the base cart: 

* The base cart uses an Raspberry Pi Pico(for onboard computation) and an NodeMCU ESP-8226(for wireless communication).
* Heavy computations are done on a desktop computer and is sent to Raspberry Pi Pico via ESP-8226.
* 4 775 DC motors are used.
* 4 custom gearbox have been created to icrease the torque of teh 775 DC motors. Gearbox images and specification are available below.
* 4 high current motor controllers are used.
* An IMU is also installed which provides accelerometer and gyroscope data. 

### Data Flow
 The following steps explain how data flows throughout the system:
 * PC Side: Initial position(0,0) is compared  with goal position and the resulting distance is fed into and mecanum drive Inverse Kinematics controller. The controller generates individual velocity reguired by each wheel to reach goal which are forwarded to the ESP-8266 via WiFi
 * ESP Side: It simply forwards the data to Pico via UART.
 * Pico Side: On the pico there are 2 cores. Core 0 is resposible for receiving and sending data to ESP. It is also responsible for estimating robot position based on sensor data using an Extended Kalman Filter. Core 1 is responsible for collecting data from 4 encoders and an IMU and sending  data to core 0 as soon as an interrupt is generated. Core 1 is kept reserved for sensor data collection.
 * Once the robot position is estimated, the pico sends  data to ESP which inturn forwards it to the PC.
    
![data_flow](https://github.com/user-attachments/assets/cde7371a-3ae4-4544-ac8e-68a7ad1fd733)

>  [!NOTE]
> In the image above, the ESP is intentially ommitted to avoid clutter.  



https://github.com/user-attachments/assets/3dc5d377-4991-4fca-b68a-abccd1728759


https://github.com/user-attachments/assets/2d0c9a70-ad9f-44bf-8a4f-b752a573428f



