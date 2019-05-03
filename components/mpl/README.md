# InvenSense Embedded Motion Driver
Ported to the ESP32 by Matt Young

Big shout out to InvenSense's management for deciding to make all the useful stuff like mag calibration, gyro calibration and 9-axis sensor fusion proprietary blobs that only run on MSP430 and ARM devices, which of course won't work on the ESP32. Hence, this component contains all the parts of the MPL that are distributed in source code and have been slightly modified to run on the ESP32.

Everything in this folder is licensed under InvenSense's license, see License.txt