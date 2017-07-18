set_global_assignment -entity "JTAG_UART" -library "lib_JTAG_UART" -name IP_TOOL_NAME "Qsys"
set_global_assignment -entity "JTAG_UART" -library "lib_JTAG_UART" -name IP_TOOL_VERSION "16.1"
set_global_assignment -entity "JTAG_UART" -library "lib_JTAG_UART" -name IP_TOOL_ENV "Qsys"
set_global_assignment -library "lib_JTAG_UART" -name SPD_FILE [file join $::quartus(sip_path) "../JTAG_UART.spd"]
set_global_assignment -library "lib_JTAG_UART" -name MISC_FILE [file join $::quartus(sip_path) "../JTAG_UART.qsys"]

set_global_assignment -library "lib_JTAG_UART" -name MISC_FILE [file join $::quartus(sip_path) "JTAG_UART.v"]
set_global_assignment -library "lib_JTAG_UART" -name MISC_FILE [file join $::quartus(sip_path) "submodules/altera_reset_controller.v"]
set_global_assignment -library "lib_JTAG_UART" -name MISC_FILE [file join $::quartus(sip_path) "submodules/altera_reset_synchronizer.v"]
set_global_assignment -library "lib_JTAG_UART" -name MISC_FILE [file join $::quartus(sip_path) "submodules/altera_reset_controller.sdc"]
set_global_assignment -library "lib_JTAG_UART" -name MISC_FILE [file join $::quartus(sip_path) "submodules/JTAG_UART_jtag_uart.v"]
