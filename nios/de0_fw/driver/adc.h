#pragma once

#define  ADC_OK            0x0000

#define  ADC_INT_DONE      0x01
#define  ADC_INT_PKT       0x02
#define  ADC_INT_ALL       0x04

// Number of 1K packets to collect before
// issuing packet interrupt.
#define  ADC_POOL_CNT      1

// Number of 1K packets to collect before
// incrementing the head address, this value
// is directly related to the com port
// pipe message transfer size. For example
// the FIFO single pipe message transfer size
// is 8K, so 8 would be used here. The OPTO
// single pipe message transfer size is 1K,
// so 1 would be used here.
#define  ADC_XFER_SIZE     1

//
// ADC Sampling Rate, based on throughput
// of hardware interface.
//
// For the ADC128S022, All 8 channels are sampled.
// This macro controls the SCLK period for the sample
// transfer which requires 16 clocks for a 12-Bit sample.
// In order to fill a 1K pipe message there are 496
// 16-Bit (only 12-bits used) samples per message.
//
// As an example, when operating an async COM port
// at 115200 baud implies 115200/10 or 11.52K bytes
// per second. A conservative transfer might be 496
// times 5 or 2480 samples (4960 bytes plus overhead)
// 5 pipe messages is 5K or 5120 bytes.
// In order to achieve this rate 2480 x 16 = 39680
// SCLKs to transfer these samples per second or
// 25.20 us SCLK period. Hence the macro value is
// the number of 10 ns FPGA clocks per SCLK period.
//
#define  ADC_SAM_RATE      2520

#define  ADC_FIFO_BASE     SDRAM_FIFO_REGION_BASE
#define  ADC_FIFO_SPAN     SDRAM_FIFO_REGION_SPAN

// ADC Interrupt Request Register
typedef union _adc_int_reg_t {
   struct {
      uint32_t done     : 1;  // adc_INT(0)
      uint32_t pkt      : 1;  // adc_INT(1)
      uint32_t          : 30;
   } b;
   uint32_t i;
} adc_int_reg_t, *padc_int_reg_t;

// ADC Control Register
typedef union _adc_ctl_reg_t {
   struct {
      uint32_t                  : 22; // adc_CONTROL(21:0)
      uint32_t ch_sel           : 4;  // adc_CONTROL(25:22)
      uint32_t ch_all           : 1;  // adc_CONTROL(26)
      uint32_t ramp             : 1;  // adc_CONTROL(27)
      uint32_t run              : 1;  // adc_CONTROL(28)
      uint32_t pkt_int          : 1;  // adc_CONTROL(29)
      uint32_t done_int         : 1;  // adc_CONTROL(30)
      uint32_t enable           : 1;  // adc_CONTROL(31)
   } b;
   uint32_t i;
} adc_ctl_reg_t, *padc_ctl_reg_t;

// ADC Status Register
typedef union _adc_sta_reg_t {
   struct {
      uint32_t head_addr        : 16; // adc_STATUS(15:0)
      uint32_t tail             : 4;  // adc_STATUS(19:16)
      uint32_t head             : 4;  // adc_STATUS(23:20)
      uint32_t                  : 6;  // adc_STATUS(29:21)
      uint32_t dma_busy         : 1;  // adc_STATUS(30)
      uint32_t adc_busy         : 1;  // adc_STATUS(31)
   } b;
   uint32_t i;
} adc_sta_reg_t, *padc_sta_reg_t;

// All Registers
typedef struct _adc_regs_t {
   uint32_t       ctl;
   uint32_t       version;
   uint32_t       irq;
   uint32_t       sta;
   uint32_t       addr_beg;
   uint32_t       addr_end;
   uint32_t       pkt_cnt;
   uint32_t       pool_cnt;
   uint32_t       adc_rate;
   uint32_t       xfer_size;
} adc_regs_t, *padc_regs_t;

uint32_t adc_init(void);
void     adc_isr(void *arg);
void     adc_intack(uint8_t int_type);
void     adc_run(uint32_t flags, uint32_t packets);
uint32_t adc_version(void);
