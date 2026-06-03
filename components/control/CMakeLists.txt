idf_component_register(SRCS "position.c" "flow_hold.c" "altitude.c" "commander.c" "pid.c" "mixer.c"
                    INCLUDE_DIRS "."
                    REQUIRES json drivers)
