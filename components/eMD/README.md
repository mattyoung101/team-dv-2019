# InvenSense Embedded Motion Driver
Ported to the ESP32 by Matt Young

Note: this component only contains InvenSense's low-level MPU driver and DMP driver. All the non-binary bits of the MPL are located in the component called "mpl". It also contains some of our own wrappers and test code. If that naming scheme doesn't make sense, don't worry, it's just too hard to change now.

dmpKey, dmpmap, inv_mpu_dmp_motion_driver and inv_mpu are all licensed under InvenSense's license, see License.txt. Everything else, like mpu_wrapper.c, is under our license.