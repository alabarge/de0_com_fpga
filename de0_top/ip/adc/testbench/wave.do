onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -expand -group SLAVE /adc_top_tb/clk
add wave -noupdate -expand -group SLAVE /adc_top_tb/reset_n
add wave -noupdate -expand -group SLAVE /adc_top_tb/irq
add wave -noupdate -expand -group SLAVE -radix hexadecimal /adc_top_tb/address
add wave -noupdate -expand -group SLAVE /adc_top_tb/read_n
add wave -noupdate -expand -group SLAVE -radix hexadecimal /adc_top_tb/readdata
add wave -noupdate -expand -group SLAVE /adc_top_tb/write_n
add wave -noupdate -expand -group SLAVE -radix hexadecimal /adc_top_tb/writedata
add wave -noupdate -expand -group SLAVE -radix hexadecimal /adc_top_tb/ADC_TOP_I/ADC_CORE_I/ADC_REGS_I/adc_ADR_BEG
add wave -noupdate -expand -group SLAVE -radix hexadecimal /adc_top_tb/ADC_TOP_I/ADC_CORE_I/ADC_REGS_I/adc_ADR_END
add wave -noupdate -expand -group SLAVE -radix hexadecimal /adc_top_tb/ADC_TOP_I/ADC_CORE_I/ADC_REGS_I/adc_PKT_CNT
add wave -noupdate -expand -group SLAVE -radix hexadecimal /adc_top_tb/ADC_TOP_I/ADC_CORE_I/ADC_REGS_I/adc_CONTROL
add wave -noupdate -expand -group MASTER -radix hexadecimal /adc_top_tb/m1_wr_address
add wave -noupdate -expand -group MASTER -radix hexadecimal /adc_top_tb/m1_writedata
add wave -noupdate -expand -group MASTER /adc_top_tb/m1_write
add wave -noupdate -expand -group MASTER /adc_top_tb/m1_wr_waitreq
add wave -noupdate -expand -group MASTER -radix hexadecimal /adc_top_tb/m1_wr_burstcount
add wave -noupdate -expand -group CHIP /adc_top_tb/cs
add wave -noupdate -expand -group CHIP /adc_top_tb/din
add wave -noupdate -expand -group CHIP /adc_top_tb/dout
add wave -noupdate -expand -group CHIP /adc_top_tb/sclk
add wave -noupdate -expand -group CHIP -radix hexadecimal /adc_top_tb/ramp
add wave -noupdate -expand -group ADC /adc_top_tb/ADC_TOP_I/ADC_CORE_I/ADC_CTL_I/ad
add wave -noupdate -expand -group ADC /adc_top_tb/ADC_TOP_I/ADC_CORE_I/ADC_CTL_I/wr
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {68405485 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 205
configure wave -valuecolwidth 72
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ms
update
WaveRestoreZoom {0 ps} {572325888 ps}
