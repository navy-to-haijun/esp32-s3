
set(src)
file(GLOB src ./eMPL/*.c)

set(dir "eMPL/" ".")
idf_component_register(SRCS ${src} "mpu6050.c" "udp_client.c"
                    INCLUDE_DIRS ${dir}
                    REQUIRES driver
                    REQUIRES freertos
                    REQUIRES esp_wifi)