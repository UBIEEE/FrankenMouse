cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32WB55xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../STM32_WPAN/App
    ../../Drivers/STM32WBxx_HAL_Driver/Inc
    ../../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy
    ../../Utilities/lpm/tiny_lpm
    ../../Middlewares/ST/STM32_WPAN
    ../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread
    ../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl
    ../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci
    ../../Middlewares/ST/STM32_WPAN/utilities
    ../../Middlewares/ST/STM32_WPAN/ble/core
    ../../Middlewares/ST/STM32_WPAN/ble/core/auto
    ../../Middlewares/ST/STM32_WPAN/ble/core/template
    ../../Middlewares/ST/STM32_WPAN/ble/svc/Inc
    ../../Middlewares/ST/STM32_WPAN/ble/svc/Src
    ../../Drivers/CMSIS/Device/ST/STM32WBxx/Include
    ../../Utilities/sequencer
    ../../Middlewares/ST/STM32_WPAN/ble
    ../../Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/app_entry.c
    ../../Core/Src/app_debug.c
    ../../Core/Src/hw_timerserver.c
    ../../Core/Src/stm32_lpm_if.c
    ../../Core/Src/stm32wbxx_it.c
    ../../Core/Src/stm32wbxx_hal_msp.c
    ../../STM32_WPAN/Target/hw_ipcc.c
    ../../STM32_WPAN/App/app_ble.c
    ../../STM32_WPAN/App/custom_stm.c
    ../../STM32_WPAN/App/custom_app.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_adc.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_adc_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_adc.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rcc.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_rcc.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rcc_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_flash.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_flash_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_gpio.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_hsem.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_dma.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_dma_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pwr.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pwr_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_cortex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_exti.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_i2c.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_i2c_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_ipcc.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_lptim.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rtc.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rtc_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_tim.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_tim_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pcd.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pcd_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_usb.c
    ../../Core/Src/system_stm32wbxx.c
    ../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.c
    ../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/shci.c
    ../../Middlewares/ST/STM32_WPAN/utilities/dbg_trace.c
    ../../Middlewares/ST/STM32_WPAN/utilities/otp.c
    ../../Middlewares/ST/STM32_WPAN/utilities/stm_list.c
    ../../Middlewares/ST/STM32_WPAN/utilities/stm_queue.c
    ../../Middlewares/ST/STM32_WPAN/ble/core/template/osal.c
    ../../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_events.c
    ../../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gap_aci.c
    ../../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gatt_aci.c
    ../../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hal_aci.c
    ../../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hci_le.c
    ../../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_l2cap_aci.c
    ../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/hci_tl.c
    ../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/hci_tl_if.c
    ../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl.c
    ../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl_if.c
    ../../Middlewares/ST/STM32_WPAN/ble/svc/Src/svc_ctl.c
    ../../Utilities/lpm/tiny_lpm/stm32_lpm.c
    ../../Utilities/sequencer/stm32_seq.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32wb55xx_cm4.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


