set(COMPONENT_ADD_INCLUDEDIRS "core/inc/"
                              "platform/inc/")

set(COMPONENT_SRCS "core/src/vl53l0x_api_calibration.c"
                    "core/src/vl53l0x_api_core.c"
                    "core/src/vl53l0x_api_ranging.c"
                    "core/src/vl53l0x_api_strings.c"
                    "core/src/vl53l0x_api.c"
                    "platform/src/vl53l0x_platform_log.c"
                    "platform/src/vl53l0x_platform.c")
set(COMPONENT_PRIV_REQUIRES "main")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D LOG_LOCAL_LEVEL=ESP_LOG_DEBUG")

register_component()
