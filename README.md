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
