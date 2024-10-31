set(src)
# file(GLOB src ./*.c)
idf_component_register(SRCS "app_main.c"
                    INCLUDE_DIRS "."
                   )
