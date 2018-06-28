# Load RUCKUS library
source -quiet $::env(RUCKUS_DIR)/vivado_proc.tcl

# Load Source Code
loadSource -dir  "$::DIR_PATH/rtl/"

if { $::env(APP_CORE_TYPE_BAY0) == "Cavity" } {
    loadRuckusTcl    "$::DIR_PATH/configCavity"
} else {
    loadRuckusTcl    "$::DIR_PATH/configStripl"
}

