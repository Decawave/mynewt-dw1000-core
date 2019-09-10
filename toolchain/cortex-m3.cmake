# Name of the target
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR armv7s)

set(MCPU_FLAGS "-mthumb -mcpu=cortex-m3")
set(VFP_FLAGS "")
set(LD_FLAGS "-nostartfiles")

include(${CMAKE_CURRENT_LIST_DIR}/common/arm-none-eabi.cmake)
