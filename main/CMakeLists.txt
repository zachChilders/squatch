# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES driver esp_timer)
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.cpp" 
                   "gnss/gnss.cpp"
                   "gnss/uart.cpp" 
                   "gnss/nmea.cpp"
                   "gnss/uart_mock.cpp"
                   "imu/mock.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()

# Enable mock modes
target_compile_definitions(${COMPONENT_LIB} PRIVATE CONFIG_IMU_MOCK CONFIG_GNSS_MOCK)