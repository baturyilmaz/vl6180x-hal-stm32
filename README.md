# VL6180X Library for STM32 (HAL)

This repository is a modified version of the Pololu Arduino library for the VL6180X distance and ambient light sensor, adapted to work with STM32 microcontrollers using the STM32 HAL Library. Note: The STM32-compatible code is at [stm32-hal-port branch](https://github.com/baturyilmaz/vl6180x-hal-stm32/tree/stm32-hal-port).


## Summary

This library helps interface with ST's VL6180X time-of-flight distance and ambient light sensor. It provides an easy way to configure the sensor and read distance and ambient light data via I²C, compatible with STM32 development environments using the HAL Library.  

## Supported Platforms

This version of the library is designed to work with STM32 microcontrollers using the STM32Cube HAL Library.