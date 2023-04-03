# STM32-with-Raspberry-Pi

This code serves to facilitate an interaction between an STM-32 and a Raspberry Pi via I2C. The STM-32 uses FreeRTOS and works to send data received from a digital input via I2C to a Raspberry Pi 4. For this project the hardware used was:
1. STM32 NUCLEO F446RE board
2. HiLetgo IR obstacle avoidance sensor (This provides the digital input)
3. Raspberry Pi 4 Model B 2GB RAM

Load the Microcontroller code using the STM32Cube IDE and a USB A male to mini B cable.
For a C-implementation: Run the ex1.c implementation from a Raspberry Pi.
For a Python implementation: Run the i2c_com.py from a Raspberry Pi.

The address assigned to this microcontroller is 0x10 and can be changed if it confilcts with addresses of other devices.