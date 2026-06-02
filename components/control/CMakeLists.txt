idf_component_register(SRCS "altitude.c" "commander.c" "pid.c" "mixer.c"
                    INCLUDE_DIRS "."
                    REQUIRES json drivers)
