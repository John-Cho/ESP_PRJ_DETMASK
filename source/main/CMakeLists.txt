#FEATURE_mPERS_CameraOV3660

idf_component_register(SRCS "uart_events_example_main.c"
                    "ov3660.c"
                    "nt99141.c"
                    "camera.c"
                    "sensor.c"
                    "xclk.c"
                    "sccb.c"
                    "twi.c"
                    "wiring.c"
                    INCLUDE_DIRS ".")

set(COMPONENT_ADD_INCLUDEDIRS "include")