# SmashIt   🎾

## About
* **SmashIt** is a tennis playing robot designed to autonomously return incoming tennis balls.
* The robot is a **mobile base with a mounted robotic arm**.
    * **The Base:** The base is a 4 wheel mecanum drive setup.
    * **The Arm:** The arm consist of 2 joints:
      *  Joint 1 is a spherical joint that allows rotation in three degrees of freedom: X, Y, and Z.
      *  The second joint is a simple revolute joint. 
* The project is implemented in both simulation (using Gazebo) and real-world hardware, utilizing ROS2 as the middleware framework.
  
>  [!NOTE]
> This project is divided into 2 parts, Simulation and Real world. The main branch is kept empty for clarity. 
> * **Simulation:** This part is software based only and uses Gazebo as simulator. **Switch to 'simulation' branch to checkout its code.**
> * **Realworld:** This part utilize realworld hardware instead of simulation. **Switch to 'real_world' branch to checkout its code.**


## Simulation

ROS2 is utilized as middleware and Gazebo is used for simulating the robot and the environment. In simulation, there are 4 major components:

* **Ball Spawner:**
   * Responsible for spawning ball in the environment.
   * It spawn ball with a random force in x, y and z direction.
      
* **Trajectory Estimator:**
   * It is responsible for:
      * Estimating ball trajectory.
      * Calculating the goal position for the mobile base to reach to take the shot.
      * Time within which the mobile base must reach the point to take the shot.
   * It estimates the trajectory of the ball using equation of bouncing ball.
   * It choose the point in the ball trajectory which is nearest to the current robot position to reduce the distance traveled by the mobile base.
   * Once the mobile base goal, end-effector goal and the time to reach is calculated, it is published to a topic for other ROS2 nodes to use.
     
* **Player:**
   * It reads the data published by the Trajectory Estimator.    
   * It is responsible for moving the robot to the estimated position and moving the arm to the point of impact.
   * It uses mecanum drive Inverse Kinematics to calculate the amount by which each wheel should move to reach the goal.
   * It uses PID controller to reach the goal.
   * Once the mobile base reaches its goal, arm Inverse Kinematics is utilized to move arm to the point of impact.
       


### Video and Images
https://github.com/user-attachments/assets/51eb29b0-6327-4e09-ad26-5f32ef586e22

![Z1](https://github.com/user-attachments/assets/d7568273-4fcd-4f84-b591-f991d7ab67b4)
![z2](https://github.com/user-attachments/assets/b7ec2bb4-4d34-4826-b7b3-e4bb54e2fc1f)
![z3](https://github.com/user-attachments/assets/ce714e35-0799-40f8-a7fb-66933369f14f)



### Steps To Run Locally

Clone the project
```bash
  git clone https://github.com/Pratham-Pandey/smash-it.git
```
Create local tracking branch for the remote branch
```bash
  git checkout -b simulation origin/simulation
```
Go to the project directory
```bash
  cd ros2
```
Build Project
```bash
  colcon build --symlink-install
```
Source Project
```bash
  source install/setup.bash 
```
Run Launch File
```bash
  ros2 launch test_bot launch_sim.launch.py
```
Run Ball Spawner
```bash
  ros2 run new_ball spawn
```
Run Trajectory Publisher
```bash
  ros2 run new_ball talker
```
Run Player Node
```bash
  ros2 run test_bot play
```



## Real World(Work in Progress)
A real world model os "SmashIt" is also being worked upon. As of now, the base(4 wheel cart) is completed and is fully functional. It can navigate to a specified coordinate within a given time, provided the input is feasible. However, due to the absence of external references like GPS, vision, or LiDAR, positional drift occurs over time. Later, an IR-based stereo vision setup will be added as an external reference. The follwoing are the key highlits of the base cart: 

* The base cart uses an **Raspberry Pi Pico(for onboard computation)** and an **NodeMCU ESP-8226(for wireless communication)**.
* Heavy computations are done on a desktop computer and is sent to Raspberry Pi Pico via ESP-8226.
* 4x **775 DC motors** are used.
* 4x custom gearbox have been designed to increase the torque of the 775 DC motors. Gearbox images and specification are available below.
* 4x **Double BTS7960 high current motor drivers** are used.
* An **MPU-6050(IMU)** is also installed which provides accelerometer and gyroscope data.
* A 12v battery with a buck converter is used to power the Pico and ESP.

### Data Flow
 The following steps explain how data flows throughout the system:
 * **PC Side:** Initial position(0,0) of the robot is compared with goal position and the resulting distance is fed into an mecanum drive controller. The controller generates individual velocity(using Inverse Kinematics) reguired by each wheel to reach goal which are forwarded to the ESP-8266 via WiFi
 * **ESP Side:** It simply forwards the data to Pico via UART.
 * **Pico Side:** On the pico there are 2 cores with the following responsiblities:
    * **Core 0:**
       * Receiving wheel velocities from ESP and forwarding it to motor controller.
       * Generating  interrupt requesting data from Core 1.
       * Estimating robot position using Encoder and IMU readings.
       * Forwarding th estimated robot position to ESP.
    * **Core 1:**
       * Collecting data from 4 encoders and an IMU.
       * Sending  data to core 0 as soon as an interrupt is generated. Core 1 is kept reserved for sensor data collection.
         
 * Once the robot position is estimated, the pico sends data to ESP which in-turn forwards it to the PC and the process repeats.



![data_flow](https://github.com/user-attachments/assets/cde7371a-3ae4-4544-ac8e-68a7ad1fd733)

>  [!NOTE]
> In the image above, the ESP is intentionally ommitted to avoid clutter.  
 
### Gear Box
A custom gearbox is designed to increase the torque of the 775 DC motor. The details are as follows:
* **775 Base motor torque:** 0.116 N·m
* **Gearbox Ratio:** 6:1
* **Total torque output per wheel(motor + gearbox):** 0.696 N.m
  
![1000023560](https://github.com/user-attachments/assets/651167bd-2f1b-4886-853d-57bf46d4ce92)
![1000023566](https://github.com/user-attachments/assets/7e36d4db-2282-4a80-92d0-1cc0144e6667)
![CAD](https://github.com/user-attachments/assets/4edbfaf1-7a4f-4ab0-88ac-e8e1e5c6953a)


### Remote Control
A remote control is designed for teleoperating the robot, specifically for testing and debugging purposes. The details are as follows:
* **Base Board:** Arduino nano
* **Switches:** 2x Joystick Switches

  ![Remote_Control](https://github.com/user-attachments/assets/557ca4e1-884e-4a73-939a-7990b71dd165)

### Video and Images

https://github.com/user-attachments/assets/7da410a6-5ed0-419e-9d48-7f9933f0e9fe

![Top_View](https://github.com/user-attachments/assets/4ffcc9af-e3d7-4492-b779-cd6cf8139fa6)
![Front_View](https://github.com/user-attachments/assets/c4095844-9a3b-4268-93a0-7cb4f1025cf9)
![Circuit_1](https://github.com/user-attachments/assets/6a93c9da-0148-446a-96ff-261cbbd02e40)
![Circuit_2](https://github.com/user-attachments/assets/8b76f58f-3005-4d71-9386-29f7b17fb994)
![Circuit_3](https://github.com/user-attachments/assets/523b8852-a32f-4c44-b208-ea17cacd347b)



### Steps To Run Locally

Clone the project
```bash
  https://github.com/Pratham-Pandey/smash-it.git
```
Create local tracking branch for the remote branch
```bash
  git checkout -b real_world origin/real_world
```
Go to the project directory
```bash
  cd ros_2
```
Build Project
```bash
  colcon build --symlink-install
```
Source Project
```bash
  source install/setup.bash 
```
Run Launch File
```bash
  ros2 launch smash_it smashit.launch.py
```
Run Remote Controller Node
```bash
  ros2 run nano_controller start
```

>  [!NOTE]
> * Make sure the system is connected to the ESP-8266 network before running the launch file.
> * Make sure the remote Controller is connected to the system before executing the Remote Controller Node.
