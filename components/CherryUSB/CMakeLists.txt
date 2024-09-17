set(SRC_DIR_LIST)
set(INC_DIR_LIST)
file(GLOB SRC_DIR_LIST 
                  "CherryUSB/core/*.c"
                  "CherryUSB/port/dwc2/usb_dc_dwc2.c"
                  "CherryUSB/port/dwc2/usb_glue_esp.c"
                  "CherryUSB/class/cdc/*.c"
                  "CherryUSB/class/msc/*.c"
                  "CherryUSB/class/hub/*.c"
                  "CherryUSB/osal/idf/usb_osal_idf.c"
                  )
file(GLOB INC_DIR_LIST 
                 "CherryUSB/core"
                 "CherryUSB/port/dwc2"
                 "CherryUSB/class/cdc"
                 "CherryUSB/class/msc"
                 "CherryUSB/common"
                 "CherryUSB/class/hub"
                  "CherryUSB/osal/idf"
                 )
idf_component_register(SRCS             ${SRC_DIR_LIST}
                       INCLUDE_DIRS     ${INC_DIR_LIST}
                       REQUIRES         usb
                    )