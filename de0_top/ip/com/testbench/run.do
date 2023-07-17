do ftdi_run_msim_rtl_vhdl.do

vcom -2008 -work work {./ftdi_TB.vhd}

vsim -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cyclonev_ver -L rtl_work -L work -voptargs="+acc"  ftdi_TB
