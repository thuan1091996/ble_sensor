# Zephyr BLE sensor application
This application is a BLE sensor application for the Zephyr RTOS. It is designed to send advertising packets with sensor data.

## Environment
- nRF connect SDK v2.4.0
- [Seeed Xiao BLE sense](https://wiki.seeedstudio.com/XIAO_BLE/)

## Flash
🔗 [Reference](https://medium.com/@aloysiousbenoy/flash-nrf52840-using-uf2-no-more-copy-paste-75044b2160a7)

## Default sensor configuration
- Default accelerometer full-scale range: +/- 4g
- Default gyroscope full-scale range: +/- 500 dps
- Default accelerometer output data: 52 Hz
- Default gyroscope output data: 52 Hz

## BLE advertising packet
|  | **Header** | **Frame Counter** | **ACL X** | **ACL Y** | **ACL Z** | **MAG X** | **MAG Y** | **MAG Z** |
|  --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **Length** | 1 | 4 | 4 | 4 | 4 | 4 | 4 | 4 |
|  |  |  |  |  |  |  |  |  |

> **Header**: 0xAA\
> **Frame Counter**: 32-bit unsigned integer\
> **ACL**: 32-bit float\
> **MAG**: 32-bit float