#pragma once

#define  ADC_DEV_NAME      "adc"

#define  ADC_OK            0x0000

#define  ADC_INT_DONE      0x01
#define  ADC_INT_PKT       0x02
#define  ADC_INT_ALL       0x04

// This address passes through the HPS windowed bridge
#define  ADC_FIFO_BASE     CP_DMA_BASE
#define  ADC_FIFO_SPAN     0x00100000

// Number of 1K packets to collect before
// issuing packet interrupt. This is also
// the rate at which the adc_head addresses
// is incremented.
#define  ADC_POOL_CNT      1

// ADC Sampling Rate, based on throughput
// of com interface.
#define  ADC_SAM_RATE      DAQ_RATE_MIN

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
      uint32_t pkt_int_en       : 1;  // adc_CONTROL(29)
      uint32_t done_int_en      : 1;  // adc_CONTROL(30)
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
} adc_regs_t, *padc_regs_t;

uint32_t adc_init(void);
void     adc_intack(uint8_t int_type);
void     adc_run(uint32_t flags, uint32_t packets);
uint32_t adc_version(void);
void     adc_final(void);
