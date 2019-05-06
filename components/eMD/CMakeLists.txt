set(COMPONENT_ADD_INCLUDEDIRS .)

set(COMPONENT_SRCS  "esp_utils.c"
                    "inv_mpu_dmp_motion_driver.c"
                    "inv_mpu.c"
                    "simple_imu.c"
                    "mpu_wrapper.c"
                    "MadgwickAHRS.c")
set(COMPONENT_PRIV_REQUIRES "main" "mpl")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D LOG_LOCAL_LEVEL=ESP_LOG_DEBUG")

register_component()
