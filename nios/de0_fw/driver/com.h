#pragma once

#include "cm.h"

#define  COM_START_FRAME      0x7E
#define  COM_END_FRAME        0x7D
#define  COM_ESCAPE           0x7C
#define  COM_STUFFED_BIT      0x20

#define  COM_INT_RX           0x01
#define  COM_INT_TX           0x02
#define  COM_INT_PIPE         0x04
#define  COM_INT_ALL          0x08

#define  COM_OK               0x00000000
#define  COM_ERROR            0x80000001
#define  COM_ERR_MSG_NULL     0x80000002
#define  COM_ERR_LEN_NULL     0x80000004
#define  COM_ERR_LEN_MAX      0x80000008
#define  COM_ERR_FRAMING      0x80000010
#define  COM_ERR_OVERRUN      0x80000020
#define  COM_ERR_PARITY       0x80000040
#define  COM_ERR_TX_DROP      0x80000080
#define  COM_ERR_RX_DROP      0x80000100
#define  COM_ERR_CRC          0x80000200
#define  COM_ERR_OPEN         0x80000400

#define  COM_OP_START         0x00000001
#define  COM_OP_STOP          0x00000002
#define  COM_OP_BYPASS        0x00000004

#define  COM_MSGLEN_UINT8     512
#define  COM_MSGLEN_UINT32    (COM_MSGLEN_UINT8 >> 2)

#define  COM_TX_BUFFER_LEN    2048
#define  COM_RX_BUFFER_LEN    2048

#define  COM_TX_QUE           8
#define  COM_RX_QUE           8

#define  COM_TX_SLOTS        (COM_TX_BUFFER_LEN / COM_MSGLEN_UINT8)
#define  COM_RX_SLOTS        (COM_RX_BUFFER_LEN / COM_MSGLEN_UINT8)

// Interrupt Register
typedef union _com_int_reg_t {
   struct {
      uint32_t tx   : 1;  // com_INT(0)
      uint32_t rx   : 1;  // com_INT(1)
      uint32_t pipe : 1;  // com_INT(2)
      uint32_t      : 29;
   } b;
   uint32_t i;
} com_int_reg_t, *pcom_int_reg_t;

// Control Register
typedef union _com_ctl_reg_t {
   struct {
      uint32_t tx_head        : 2;  // com_CONTROL(1:0)
      uint32_t rx_tail        : 2;  // com_CONTROL(3:2)
      uint32_t                : 21; // com_CONTROL(24:4)
      uint32_t pipe_int       : 1;  // com_CONTROL(25)
      uint32_t dma_req        : 1;  // com_CONTROL(26)
      uint32_t rx_int         : 1;  // com_CONTROL(27)
      uint32_t tx_int         : 1;  // com_CONTROL(28)
      uint32_t pipe_run       : 1;  // com_CONTROL(29)
      uint32_t com_run       : 1;  // com_CONTROL(30)
      uint32_t enable         : 1;  // com_CONTROL(31)
   } b;
   uint32_t i;
} com_ctl_reg_t, *pcom_ctl_reg_t;

// Status Register
typedef union _com_ctl_sta_t {
   struct {
      uint32_t tail_addr      : 16; // com_STATUS(15:0)
      uint32_t rx_head        : 2;  // com_STATUS(17:16)
      uint32_t tx_tail        : 2;  // com_STATUS(19:18)
      uint32_t                : 9;  // com_STATUS(28:20)
      uint32_t rx_rdy         : 1;  // com_STATUS(29)
      uint32_t tx_rdy         : 1;  // com_STATUS(30)
      uint32_t tx_busy        : 1;  // com_STATUS(31)
   } b;
   uint32_t i;
} com_sta_reg_t, *pcom_sta_reg_t;

// All Registers
typedef struct _com_regs_t {
   uint32_t       ctl;
   uint32_t       version;
   uint32_t       test_bit;
   uint32_t       irq;
   uint32_t       sta;
   uint32_t       addr_beg;
   uint32_t       addr_end;
   uint32_t       pktcnt;
   uint32_t       ticks;
   uint32_t       unused[503];
   uint32_t       rx_buf[512];
   uint32_t       tx_buf[512];
   uint32_t       pipe[512];
} com_regs_t, *pcom_regs_t;

// Transmit Queue
typedef struct _com_txq_t {
   uint32_t     *buf[COM_TX_QUE];
   uint8_t       tail;
   uint8_t       head;
   uint8_t       slots;
} com_txq_t, *pcom_txq_t;

// Receive Queue
typedef struct _com_rxq_t {
   uint32_t     *buf[COM_RX_QUE];
   uint8_t       tail;
   uint8_t       head;
   uint8_t       slots;
} com_rxq_t, *pcom_rxq_t;

uint32_t  com_init(uint32_t baudrate, uint8_t port);
void      com_isr(void *arg);
void      com_intack(uint8_t int_type);
void      com_tx(pcm_msg_t msg);
void      com_cmio(uint8_t op_code, pcm_msg_t msg);
void      com_msgtx(void);
void      com_pipe(uint32_t opcode, uint32_t addr_beg, uint32_t addr_end, uint32_t pktcnt);
uint32_t  com_version(void);

