# STM_Stewart
Bachelor's thesis in the Warsaw University of Technology.

![WUT logo](https://github.com/Larook/STM_Stewart/blob/final/LaTeX_thesis/Thesis_images/logopw.png)

The project's goal was to develop a 6DOF parallel manipulator known as the Stewart platform.
On top of the STM32 - based motion control an attempt to balance a metal ball on top of the platform's upper plate using the resistive touch sensor and PID controller.

Outcome:
The structure works fine with all the movements possible, combined with gimbal-like stabilisation thanks to the used IMU.
Unfortunately the inaccuracy of servos in the joints were pointed as the main obstacle preventing from succesfull ball balancing on the limited area of top plate.

![build](https://github.com/Larook/STM_Stewart/blob/final/LaTeX_thesis/Thesis_images/Czesci_i_konstrukcja/Platforma_rzeczywista.JPG)

Construction:
![inv_kin](https://github.com/Larook/STM_Stewart/blob/final/LaTeX_thesis/Thesis_images/Kinematyka_odwrotna/noga_serwo_wymiary_1.png)

![touch_panel](https://github.com/Larook/STM_Stewart/blob/final/LaTeX_thesis/Thesis_images/TouchPanel/HOW%20IT%20SHOULD%20WORK.jpg)

State machine:
![state_machine](https://github.com/Larook/STM_Stewart/blob/final/LaTeX_thesis/Thesis_images/statemachine.png)


PID response:
![state_machine](https://github.com/Larook/STM_Stewart/blob/final/LaTeX_thesis/Thesis_images/step_response.png)
