# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES driver)
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.cpp" 
                   "gnss/gnss.cpp"
                   "gnss/uart.cpp" 
                   "gnss/nmea.cpp"
                   "logging/sensor_logger.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()