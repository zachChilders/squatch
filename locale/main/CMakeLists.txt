# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES driver esp_timer)
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.cpp" 
                   "tasks.cpp"
                   "gnss/gnss.cpp"
                   "gnss/nmea.cpp"
                   "imu/mock.cpp"
                   "lib/uart.cpp"
                   "lib/uart_mock.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()

# Configurable mock modes - defaults to mock for development, use flags for hardware
option(ENABLE_IMU_MOCK "Enable IMU mock mode" ON)
option(ENABLE_GNSS_MOCK "Enable GNSS mock mode" ON)

# Check environment variables if CMake options not explicitly set
if(NOT DEFINED CACHE{ENABLE_IMU_MOCK} AND DEFINED ENV{ENABLE_IMU_MOCK})
    set(ENABLE_IMU_MOCK $ENV{ENABLE_IMU_MOCK})
endif()

if(NOT DEFINED CACHE{ENABLE_GNSS_MOCK} AND DEFINED ENV{ENABLE_GNSS_MOCK})
    set(ENABLE_GNSS_MOCK $ENV{ENABLE_GNSS_MOCK})
endif()

# Apply compile definitions based on configuration
if(ENABLE_IMU_MOCK)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE CONFIG_IMU_MOCK)
    message(STATUS "IMU mock mode enabled")
endif()

if(ENABLE_GNSS_MOCK)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE CONFIG_GNSS_MOCK)
    message(STATUS "GNSS mock mode enabled")
endif()