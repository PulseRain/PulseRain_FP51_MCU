set_global_assignment -entity "Altera_unique_chip_ID" -library "lib_Altera_unique_chip_ID" -name IP_TOOL_NAME "altchip_id"
set_global_assignment -entity "Altera_unique_chip_ID" -library "lib_Altera_unique_chip_ID" -name IP_TOOL_VERSION "16.1"
set_global_assignment -entity "Altera_unique_chip_ID" -library "lib_Altera_unique_chip_ID" -name IP_TOOL_ENV "mwpim"
set_global_assignment -library "lib_Altera_unique_chip_ID" -name SPD_FILE [file join $::quartus(sip_path) "Altera_unique_chip_ID.spd"]

set_global_assignment -library "lib_Altera_unique_chip_ID" -name MISC_FILE [file join $::quartus(sip_path) "Altera_unique_chip_ID_sim/Altera_unique_chip_ID.v"]
set_global_assignment -library "lib_Altera_unique_chip_ID" -name MISC_FILE [file join $::quartus(sip_path) "Altera_unique_chip_ID_sim/altchip_id/altchip_id.v"]
