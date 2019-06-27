# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES "nanopb")

set(COMPONENT_SRCS "defines.c"
                   "main.c"
                   "utils.c"
                   "comms_i2c.c"
                   "tsop.c"
                   "motor.c"
                   "pid.c"
                   "fsm.c"
                   "states_attack.c"
                   "states_defence.c"
                   "states_general.c"
                   "cam.c"
                   "music.c"
                   "mplexer.c"
                   "movavg.c"
                   "comms_bluetooth.c"
                   "bluetooth_manager.c"
                   "lrf.c"
                   )
set(COMPONENT_ADD_INCLUDEDIRS .)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D LOG_LOCAL_LEVEL=ESP_LOG_DEBUG")

register_component()
