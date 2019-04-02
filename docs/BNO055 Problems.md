# Clock stretching solutions

### Solution 1
```c
I2C[i2c_num]->timeout.tout = 200000;
in driver/i2c.c - i2c_param_config
```

[Source](http://bbs.esp32.com/viewtopic.php?t=4242#p40483)

## Solution 2 (the current one)
Just call `i2c_set_timeout(I2C_NUM_1, 640000);`

> Seems to destroy the I2C comms from master to slave though

## Solution 3
Slow down clock rate to like 1000 Hz

> Very bad, may as well use UART in that case!

## Notes/links
- According to [this post](https://forums.adafruit.com/viewtopic.php?f=19&t=126665#p632862), the MPU9250 is faster than the BNO
- https://os.mbed.com/users/kenjiArai/notebook/bno055---orientation-sensor/
- https://forum.digilentinc.com/topic/1982-issue-with-bno055-sensor-on-uc32-via-i2c/
- https://github.com/espressif/arduino-esp32/issues/81 (good post)
- https://github.com/espressif/arduino-esp32/issues/69 (seems to be fixed?)
- https://github.com/espressif/esp-idf/issues/2551 (WIP)
- Pls buy logic analyser, it would be cool and useful