transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vcom -2008 -work work {../../../packages/de10_pkg.vhd}
vcom -2008 -work work {../../../packages/PCK_tb.vhd}
vcom -2008 -work work {../../../packages/PCK_print_utilities.vhd}
vcom -2008 -work work {../../../packages/PCK_FIO_1993.vhd}
vcom -2008 -work work {../../../packages/PCK_FIO_1993_BODY.vhd}
vcom -2008 -work work {../ftdi_2k.vhd}
vcom -2008 -work work {../ftdi_burst.vhd}
vcom -2008 -work work {../ftdi_fifo.vhd}
vcom -2008 -work work {../ftdi_rtx.vhd}
vcom -2008 -work work {../ftdi_ctl.vhd}
vcom -2008 -work work {../ftdi_irq.vhd}
vcom -2008 -work work {../ftdi_regs.vhd}
vcom -2008 -work work {../ftdi_core.vhd}
vcom -2008 -work work {../ftdi_top.vhd}
