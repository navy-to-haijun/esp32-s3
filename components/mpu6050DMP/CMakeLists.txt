
set(src)
file(GLOB src ./eMPL/*.c)

set(dir "eMPL/" ".")
idf_component_register(SRCS ${src} "mpu6050.c"
                    INCLUDE_DIRS ${dir}
                    REQUIRES driver
                    REQUIRES freertos)