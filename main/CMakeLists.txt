set(src)
# file(GLOB src ./*.c)
idf_component_register(SRCS  "app_main.c" "msc_storage_template.c"
                    INCLUDE_DIRS "."
                   )
