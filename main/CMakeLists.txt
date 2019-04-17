# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.c"
                   "comms_i2c.c"
                   "tsop.c"
                   "utils.c"
                   "motor.c"
                   "pid.c"
                   "light.c"
                   "fsm.c"
                   "states_attack.c"
                   "states_defence.c"
                   "states_general.c"
                   "cam.c"
                   "music.c"
                   "mplexer.c"
                   "comms_wifi.c"
                   "movavg.c"
                   )
set(COMPONENT_ADD_INCLUDEDIRS .)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D LOG_LOCAL_LEVEL=ESP_LOG_DEBUG")

register_component()
