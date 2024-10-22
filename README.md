# Introduction
This is the codebase for my Master's thesis in Mechanical Engineering. For this project, I developed:
- A four-stacked hexapod platform, with each platform capable of independent control, used as a ground-truth positioning system
- An ultrasonic, air-based positioning system consisting of four receivers (with active band-pass filtering) and a transmitter
- A method for transmitting 2kb/s of data over ultrasonic sound
- And much more!

My thesis is available at [Cal Poly's Digital Commons](https://digitalcommons.calpoly.edu/theses/2914/) and on my [LinkedIn profile](https://www.linkedin.com/in/jakobhadal/). You can see a [video of the system in action here](https://www.youtube.com/watch?v=QgJyHS0l2EE).

The hardware used includes an ESP32-WROOM (hexapod platform), STM32H723 (acoustic positioning algorithm and sensor fusion), and STM32F411 (acoustic transmitter). The ESP32 was programmed using PlatformIO and C++, and the STM32s were programmed using STM32CubeIDE and C. Some data analysis was performed using Python.

# Abstract
This document details the design, implementation, testing, and analysis of an inverted short baseline acoustic positioning system. The system presented here is an above-water, air-based prototype for an underwater acoustic positioning system; it is designed to determine the position of remotely-operated underwater vehicles (ROVs) and autonomous underwater vehicles (AUVs) in the global frame using a method that does not drift over time.
		
A ground-truth positioning system is constructed using a stacked hexapod platform actuator, which mimics the motion of an AUV and provides the true position of an ultrasonic microphone array. An ultrasonic transmitter sends a pulse of sound towards the array; microphones on the array record the pulse of sound and use the time shift between the microphone signals to determine the position of the transmitter relative to the receiver array. The orientation of the array, which is necessary to transform the position estimate to the global frame, is calculated using a Madgwick filter and data from a MEMS IMU. Additionally, a dead reckoning change-in-position estimate is formed using the IMU data. The acoustic position estimate is combined with the dead reckoning estimate using a Kalman filter. The accuracy of this filtered position estimate was verified to 22.1mm within a range of 3.88m in this air-based implementation. The ground-truth positioning system runs on an ESP32 microcontroller using code written in C++, and the acoustic positioning system runs on two STM32 microcontrollers using code written in C.
		  
Extrapolation of these results to the underwater regime, as well as recommendations for improving upon this work, are included at the end of the document. All code written for this thesis is available on GitHub and is open-source and well-documented.
