cmake_minimum_required(VERSION 3.13)

project(opencm3 C CXX)

message("Build type is ${CMAKE_BUILD_TYPE}")

set(CMAKE_UNITY_BUILD ON)

string(REPLACE "\"" "" FILT_DEVICES ${DEVICES} TARGET_DEFINITION)
foreach(TARGET_DEVICE ${DEVICES})
    message(STATUS "Generating build environment for: ${TARGET_DEVICE}")
    file (GLOB_RECURSE CORTEX_M_SOURCES CONFIGURE_DEPENDS lib/cm3/*.c)
    file (GLOB_RECURSE DEVICE_SOURCES CONFIGURE_DEPENDS lib/${TARGET_DEVICE}/*.c)
    list (REMOVE_ITEM DEVICE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/lib/${TARGET_DEVICE}/vector_nvic.c)

    add_library(opencm3_${TARGET_DEVICE} STATIC
        ${CORTEX_M_SOURCES}
        ${DEVICE_SOURCES}
        )

    set_target_properties(opencm3_${TARGET_DEVICE} PROPERTIES
        UNITY_BUILD_MODE GROUP
        )

    set_source_files_properties(${CORTEX_M_SOURCES} ${DEVICE_SOURCES}
        PROPERTIES UNITY_GROUP "bucket1"
        )

    string(TOUPPER ${TARGET_DEVICE} TARGET_DEFINITION)
    target_compile_definitions(opencm3_${TARGET_DEVICE} PRIVATE
        -D ${TARGET_DEFINITION}
#        -D __ARM_ARCH_7M__
        )

    target_include_directories(opencm3_${TARGET_DEVICE} PUBLIC include)

    execute_process(
        COMMAND ${Python3_EXECUTABLE} "./scripts/irq2nvic_h" "./include/libopencm3/${TARGET_DEVICE}/irq.json"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        RESULT_VARIABLE py_result
        )

endforeach()
