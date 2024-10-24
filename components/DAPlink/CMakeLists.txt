set(SRC_DIR_LIST)
set(INC_DIR_LIST)
file(GLOB SRC_DIR_LIST 
                  "Source/*.c"
                  )
file(GLOB INC_DIR_LIST 
                 "Include"
                 "Config"
                 )
idf_component_register(SRCS             ${SRC_DIR_LIST}
                       INCLUDE_DIRS     ${INC_DIR_LIST}
                       REQUIRES         usb
                    )