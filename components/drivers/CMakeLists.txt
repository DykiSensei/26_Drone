idf_component_register(SRCS "motor.c" "pv3901l1.c" "tof400f.c" "mpu6050.c" "i2c_bus.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer freertos)
