set(COMPONENT_ADD_INCLUDEDIRS .)

set(COMPONENT_SRCS  "data_builder.c"
                    "eMPL_outputs.c"
                    "hal_outputs.c"
                    "message_layer.c"
                    "ml_math_func.c"
                    "mlmath.c"
                    "mpl.c"
                    "results_holder.c"
                    "start_manager.c"
                    "storage_manager.c")
set(COMPONENT_PRIV_REQUIRES "main")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D LOG_LOCAL_LEVEL=ESP_LOG_DEBUG")

register_component()
