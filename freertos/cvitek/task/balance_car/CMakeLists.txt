file(GLOB _SOURCES "src/*.c")

include_directories(include)
include_directories(${CMAKE_INSTALL_INC_PREFIX}/arch)
include_directories(${CMAKE_INSTALL_INC_PREFIX}/common)
include_directories(${CMAKE_INSTALL_INC_PREFIX}/kernel)
include_directories(${CMAKE_INSTALL_INC_PREFIX}/driver/gpio)
include_directories(${CMAKE_INSTALL_INC_PREFIX}/driver/pinmux)
include_directories(${CMAKE_INSTALL_INC_PREFIX}/hal/config)
include_directories(${CMAKE_INSTALL_INC_PREFIX}/hal/pinmux)

add_library(balance_car STATIC ${_SOURCES})
install(TARGETS balance_car DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/task/balance_car)
