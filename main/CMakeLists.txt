# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES driver)
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.cpp" 
                   "gnss/gnss.cpp"
                   "gnss/uart.cpp" 
                   "gnss/nmea.cpp"
                   "imu/mock.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()

# Enable IMU mock mode
target_compile_definitions(${COMPONENT_LIB} PRIVATE CONFIG_IMU_MOCK)