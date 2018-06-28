# Load RUCKUS library
source -quiet $::env(RUCKUS_DIR)/vivado_proc.tcl

loadSource -path "$::DIR_PATH/rtl/AppCoreConfigPkg.vhd"
