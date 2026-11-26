
# Source Localization System — Direction of Arrival Estimation

An embedded Direction of Arrival (DOA) estimation system based on the ESP32-S3 microcontroller and four SPH0645LM4H digital MEMS microphones. The firmware, written in C/C++, processes acoustic signals directly on the microcontroller and estimates the direction of the sound source within a 0–360° range.

The project was developed as part of **AGH Drone Engineering**. The system uses four microphones arranged at the corners of a square. A custom four-layer PCB has been designed, manufactured, assembled and successfully brought up to integrate the system into a single hardware module.

## Key Features

- Simultaneous acquisition of two microphone pairs using two independent I²S controllers available on the ESP32-S3.
- Time Difference of Arrival (TDOA) estimation using the Generalized Cross-Correlation with Phase Transform (GCC-PHAT) algorithm for each synchronized microphone pair.
- Direction of Arrival (DOA) estimation based on microphone array geometry and an acoustic wave propagation model.
- Rejection of unreliable measurements and circular averaging of consecutive angle estimates.
- Serial communication for system control and transmission of estimated angles in the `ANGLE <angle>` format.
- Dedicated diagnostic mode for acquiring raw audio samples from all four microphones.

## Hardware and Software Architecture

- **Microcontroller:** ESP32-S3
- **Microphones:** 4 × SPH0645LM4H digital MEMS microphones with an I²S interface
- **Microphone arrangement:** Two microphones along each diagonal of a square
- **Development environment:** PlatformIO
- **Framework:** Arduino
- **Programming languages:** C/C++
- **Hardware platform:** Custom four-layer PCB

## Audio Recording from the PCB via USB-C

Run the following commands from the repository root directory:

```bash
~/.platformio/penv/bin/pio run -e esp32-s3-recorder -t upload
~/.platformio/penv/bin/python pc_tools/record_mics.py --seconds 10
```

Close the Serial Monitor before starting the recording.

Individual microphone recordings (`MK1.wav`–`MK4.wav`) and a combined recording (`mikrofony.wav`) are saved in the `nagrania/<timestamp>/` directory.

Audio recordings use a sampling rate of **16 kHz** and **16-bit PCM** encoding.

The Python script automatically starts and stops audio acquisition.

PCB pin assignments and localization mode configuration are documented in [LOCALIZATION.md](LOCALIZATION.md).
