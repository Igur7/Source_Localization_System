
# Source Localization System — Direction of Arrival Estimation

An embedded Direction of Arrival (DOA) estimation system based on the ESP32-S3 microcontroller and four SPH0645LM4H digital MEMS microphones. The firmware, written in C/C++, processes acoustic signals directly on the microcontroller and estimates the direction of the sound source within a 0–360° range.

The project was developed as part of **AGH Drone Engineering**. The system uses four microphones arranged at the corners of a square. A custom four-layer PCB, developed specifically for this application, has been designed, assembled and successfully brought up.

## Key Features

- Simultaneous acquisition of two microphone pairs using two independent I²S controllers available on the ESP32-S3.
- Time Difference of Arrival (TDOA) estimation using the Generalized Cross-Correlation with Phase Transform (GCC-PHAT) algorithm for each synchronized microphone pair.
- Direction of Arrival (DOA) estimation based on microphone array geometry and an acoustic wave propagation model.
- Rejection of unreliable measurements and circular averaging of consecutive angle estimates.
- Serial communication for system control and transmission of estimated angles in the `ANGLE <angle>` format.
- Dedicated diagnostic mode for acquiring raw audio samples from all four microphones.

## Hardware and Software Architecture

- **Microcontroller:** ESP32-S3
- **Microphones:** 4 × SPH0645LM4H digital MEMS microphones
- **Communication interface:** I²S
- **Microphone arrangement:** Two microphone pairs positioned along the diagonals of a square
- **Development environment:** PlatformIO
- **Framework:** Arduino
- **Programming languages:** C/C++
- **Signal processing:** GCC-PHAT, TDOA and DOA estimation
- **Hardware platform:** Custom four-layer PCB

## Project Status

The sound source localization prototype has been developed and tested. The custom PCB has been manufactured, assembled and successfully brought up, integrating the microcontroller, microphone array and supporting electronics into a single hardware module.
