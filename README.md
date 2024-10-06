  A sensor that checks and returns if the toilet lock is locked when receives signal from master device.

  Sensor code runs on Arduino Nano. A VL53L0X range sensor is used to measure distance of lock pin and check whether it is locked or not.
Communication between master device and sensor is established using HC-05 Bluetooth module. Since sensor can work remotely, it doesn't need a connection to computer.

  Master code runs on Arduino Uno. Signals are created by a button connected to Arduino Uno and are sent to sensor by HC-05 Bluetooth module. The results obtained from the sensor
is printed on serial monitor, so the master device should be connected to a computer.

  First signal from master device is used to configure the distance of lock pin when unlocked. After the configuration, the sensor gets into sleep mode and waits until next signal.
When any new signal from master device comes, the sensor wakes up and measures the distance of lock pin, decides if it is locked or not, then returns the corresponding signal to master device.
