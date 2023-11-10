## Version 0.0.1 (Oct 10, 2023)
- Initial release
- BLE operation as a broadcaster
- Sensor data is sent via BLE advertising packets

## Version 0.0.2 (Oct 19, 2023)
- Change BLE operation from broadcaster to connectable peripheral
- Sensor data is sent via indication messages
- Including device MAC address in advertising field "IMUXXXXXXXXXXXX"
- Reserved "COMMAND characteristic"

## Version 0.0.3 (Nov 10, 2023)
- Fix BLE advertising after connection
- Enable auto trigger connection parameter update
- Enable power management