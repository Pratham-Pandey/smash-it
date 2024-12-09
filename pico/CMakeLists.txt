cmake_minimum_required(VERSION 3.13...3.27)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(motor_ros)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
if (TARGET tinyusb_device)
    add_executable(motor_exe
        m2.c
    )

    # pull in common dependencies
    target_link_libraries(motor_exe pico_stdlib hardware_pwm pico_multicore)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(motor_exe 1)
    pico_enable_stdio_uart(motor_exe 1)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(motor_exe)

    # add url via pico_set_program_url
    #example_auto_set_url(comm1_exe)
elseif(PICO_ON_DEVICE)
    message("Skipping hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()


