#pragma once

#include "cm.h"

#define  TTY_SOF              0x7E
#define  TTY_EOF              0x7D
#define  TTY_ESC              0x7C
#define  TTY_BIT              0x20

#define  TTY_INT_RX           0x01
#define  TTY_INT_TX           0x02
#define  TTY_INT_ALL          0x04

#define  TTY_OK               0x00000000
#define  TTY_ERROR            0x80000001
#define  TTY_ERR_MSG_NULL     0x80000002
#define  TTY_ERR_LEN_NULL     0x80000004
#define  TTY_ERR_LEN_MAX      0x80000008
#define  TTY_ERR_FRAMING      0x80000010
#define  TTY_ERR_OVERRUN      0x80000020
#define  TTY_ERR_PARITY       0x80000040
#define  TTY_ERR_TX_DROP      0x80000080
#define  TTY_ERR_RX_DROP      0x80000100
#define  TTY_ERR_CRC          0x80000200
#define  TTY_ERR_OPEN         0x80000400

#define  TTY_IDLE             0
#define  TTY_IN_MSG           1
#define  TTY_IN_ESC           2

#define  TTY_MSGLEN_UINT8     512
#define  TTY_MSGLEN_UINT32    (TTY_MSGLEN_UINT8 >> 2)

#define  TTY_TX_QUE           8
#define  TTY_RX_QUE           1

// Control Register
typedef union _tty_ctl_reg_t {
   struct {
      uint32_t ipe            : 1;  // tty_control(0)
      uint32_t ife            : 1;  // tty_control(1)
      uint32_t ibrk           : 1;  // tty_control(2)
      uint32_t iroe           : 1;  // tty_control(3)
      uint32_t itoe           : 1;  // tty_control(4)
      uint32_t itmt           : 1;  // tty_control(5)
      uint32_t itrdy          : 1;  // tty_control(6)
      uint32_t irrdy          : 1;  // tty_control(7)
      uint32_t ie             : 1;  // tty_control(8)
      uint32_t trbk           : 1;  // tty_control(9)
      uint32_t idcts          : 1;  // tty_control(10)
      uint32_t rts            : 1;  // tty_control(11)
      uint32_t ieop           : 1;  // tty_control(12)
      uint32_t                : 19; // tty_control(31:13)
   } b;
   uint32_t i;
} tty_ctl_reg_t, *ptty_ctl_reg_t;

// Status Register
typedef union _tty_ctl_sta_t {
   struct {
      uint32_t pe             : 1;  // tty_status(0)
      uint32_t fe             : 1;  // tty_status(1)
      uint32_t brk            : 1;  // tty_status(2)
      uint32_t roe            : 1;  // tty_status(3)
      uint32_t toe            : 1;  // tty_status(4)
      uint32_t tmt            : 1;  // tty_status(5)
      uint32_t trdy           : 1;  // tty_status(6)
      uint32_t rrdy           : 1;  // tty_status(7)
      uint32_t e              : 1;  // tty_status(8)
      uint32_t                : 1;  // tty_status(9)
      uint32_t dcts           : 1;  // tty_status(10)
      uint32_t cts            : 1;  // tty_status(11)
      uint32_t eop            : 1;  // tty_status(12)
      uint32_t                : 19; // tty_status(31:13)
   } b;
   uint32_t i;
} tty_sta_reg_t, *ptty_sta_reg_t;

// All Registers
typedef struct _tty_regs_t {
   uint32_t       rxd;
   uint32_t       txd;
   uint32_t       sta;
   uint32_t       ctl;
   uint32_t       div;
   uint32_t       eop;
} tty_regs_t, *ptty_regs_t;

// Transmit Queue
typedef struct _tty_txq_t {
   uint32_t     *buf[TTY_TX_QUE];
   uint32_t      len[TTY_TX_QUE];
   uint32_t      n;
   uint8_t       tail;
   uint8_t       head;
   uint8_t       slots;
   uint8_t       state;
} tty_txq_t, *ptty_txq_t;

// Receive Queue
typedef struct _tty_rxq_t {
   uint32_t      raw[TTY_MSGLEN_UINT32];
   uint8_t      *buf;
   uint32_t      n;
   uint8_t       state;
} tty_rxq_t, *ptty_rxq_t;

uint32_t  tty_init(uint32_t baudrate, uint8_t port);
void      tty_isr(void *arg);
void      tty_intack(uint8_t int_type);
void      tty_tx(void);
void      tty_cmio(uint8_t op_code, pcm_msg_t msg);
void      tty_msgtx(void);
void      tty_pipe(uint32_t index, uint32_t msglen);

