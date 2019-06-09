set(COMPONENT_ADD_INCLUDEDIRS .)

set(COMPONENT_SRCS "pb_common.c"
                   "pb_decode.c"
                   "pb_encode.c"
                   "i2c.pb.c"
                   "bluetooth.pb.c")

register_component()
